package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.SwerveConstants;

/* Sets up class that assigns motors to each swerve module and get swerving.
* Methods created to handle different actions taken on the controls.
*/
public class SwerveSubsystem extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveOdometry swerveOdometry;
    private OdometryThread odometryThread;
    private SwerveModule[] swerveModules;
    private SwerveModulePosition[] modulePositions;

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];

    private RobotConfig robotConfiguration;

    private Field2d field;

    private final SysIdRoutine sysIDRoutine;
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public enum DriveMode {
        FIELD_RELATIVE,
        ROBOT_RELATIVE
    }

    public enum SlowMode {
        ON(1),
        OFF(0.3);

        private final double speedMultiplier;

        private SlowMode(double speedMultiplier) {
            this.speedMultiplier = speedMultiplier;
        }

        public double getSpeedMultiplier() {
            return this.speedMultiplier;
        }
    }

    private DriveMode driveMode;
    private SlowMode slowMode;

    public SwerveSubsystem() {
        System.out.println("[Initialization] Creating SwerveSubsystem");

        gyro = new Pigeon2(14);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);
        gyro.getAngularVelocityZWorld().setUpdateFrequency(100);
        gyro.optimizeBusUtilization();
        resetYaw(Rotation2d.fromDegrees(0.0));
        
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.ModuleConstants.FrontLeftModule.constants),
            new SwerveModule(1, Constants.ModuleConstants.FrontRightModule.constants),
            new SwerveModule(2, Constants.ModuleConstants.BackLeftModule.constants),
            new SwerveModule(3, Constants.ModuleConstants.BackRightModule.constants)
        };

        // TODO: Add STDDevs to get vision implemented
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.SwerveKinematics, 
            getYaw(), 
            getSwerveModulePositions(), 
            new Pose2d()
        );

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.SwerveKinematics, getYaw(), getSwerveModulePositions());
        odometryThread = new OdometryThread();
        odometryThread.start();

        field = new Field2d();

        try {
            robotConfiguration = RobotConfig.fromGUISettings();
          } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getOdometryPose, 
            this::resetSwerveOdometry,
            this::getRobotRelativeSpeeds, 
            (speeds) -> setChassisSpeeds(speeds), 
            new PPHolonomicDriveController(
                Constants.AutonomousConstants.TranslationPID, 
                Constants.AutonomousConstants.RotationalPID
            ), 
            robotConfiguration,
            () -> {
                return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
            },
            this
        );

        driveMode = DriveMode.FIELD_RELATIVE;
        slowMode = SlowMode.OFF;

        sysIDRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism( // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            voltage -> {
                swerveModules[0].setDriveVoltage(voltage);
                swerveModules[1].setDriveVoltage(voltage);
                swerveModules[2].setDriveVoltage(voltage);
                swerveModules[3].setDriveVoltage(voltage);
            },
            log -> {
                log.motor("Front-Left Module")
                    .voltage(m_appliedVoltage.mut_replace(swerveModules[0].getDriveMotor() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(swerveModules[0].getModulePosition(true).distanceMeters, Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[0].getModuleState(true).speedMetersPerSecond, MetersPerSecond));
                log.motor("Front-Right Module")
                    .voltage(m_appliedVoltage.mut_replace(swerveModules[1].getDriveMotor() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(swerveModules[1].getModulePosition(true).distanceMeters, Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[1].getModuleState(true).speedMetersPerSecond, MetersPerSecond));

                log.motor("Back-Left Module")
                    .voltage(m_appliedVoltage.mut_replace(swerveModules[2].getDriveMotor() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(swerveModules[2].getModulePosition(true).distanceMeters, Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[2].getModuleState(true).speedMetersPerSecond, MetersPerSecond));

                log.motor("Back-Right Module")
                    .voltage(m_appliedVoltage.mut_replace(swerveModules[3].getDriveMotor() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(swerveModules[3].getModulePosition(true).distanceMeters, Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[3].getModuleState(true).speedMetersPerSecond, MetersPerSecond));
                }, 
            this)
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    }

    public void switchDriveMode() {
       if(this.driveMode == DriveMode.FIELD_RELATIVE) {
        this.driveMode = DriveMode.ROBOT_RELATIVE;
       } else if (this.driveMode == DriveMode.ROBOT_RELATIVE) {
        this.driveMode = DriveMode.FIELD_RELATIVE;
       }
    }

    public void switchSlowMode() {
        if(this.slowMode == SlowMode.OFF) {
            this.slowMode = SlowMode.ON;
        } else if (this.slowMode == SlowMode.ON) {
            this.slowMode = SlowMode.OFF;
        }
     }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates;

        switch(driveMode) {
            case FIELD_RELATIVE:
                swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX() * slowMode.getSpeedMultiplier(), 
                        translation.getY() * slowMode.getSpeedMultiplier(), 
                        rotation * slowMode.getSpeedMultiplier(), 
                        getYaw()
                    )
                );

                break;
            case ROBOT_RELATIVE:
                swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(
                    new ChassisSpeeds(translation.getX() * slowMode.getSpeedMultiplier(), translation.getY() * slowMode.getSpeedMultiplier(), rotation * slowMode.getSpeedMultiplier())
                );

                break;
            default:
                swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX() * slowMode.getSpeedMultiplier(), 
                        translation.getY() * slowMode.getSpeedMultiplier(), 
                        rotation * slowMode.getSpeedMultiplier(), 
                        getYaw()
                    )
                );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxVelocity);

        for(SwerveModule module : swerveModules) {
            module.set(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters, boolean isOpenLoop){
        desiredSpeeds = chassisSpeeds;
        desiredStates = SwerveConstants.SwerveKinematics.toSwerveModuleStates(desiredSpeeds, centerOfRotationMeters);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.PhysicalMaxVelocity);

        for (SwerveModule mod : swerveModules) {
            mod.set(desiredStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop){
        setChassisSpeeds(chassisSpeeds, new Translation2d(), isOpenLoop);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        setChassisSpeeds(chassisSpeeds, false);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxVelocity);

        for(SwerveModule module : swerveModules) {
            module.set(desiredStates[module.moduleNumber], false);
        }
    }

    public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode){
        for (SwerveModule module : swerveModules) {
            module.setDriveNeutralMode(driveMode);
            module.setSteerNeutralMode(steerMode);
        }
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : swerveModules) {
            states[module.moduleNumber] = module.getModuleState(true);
        }

        return states;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module : swerveModules) {
            positions[module.moduleNumber] = module.getModulePosition(true);
        }

        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.SwerveConstants.SwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }


    public Pose2d getOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Rotation2d getEstimatedHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public Rotation2d getOdometryHeading() {
        return swerveOdometry.getPoseMeters().getRotation();
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.gyroInverted) ? gyro.getRotation2d().minus(Rotation2d.fromDegrees(360)) : gyro.getRotation2d();
    }

    public void resetModulePositions(){
        for(SwerveModule module : swerveModules) {
            module.resetModulePosition();
        }
    }

    public void resetSwerveOdometry(Pose2d pose) {
        resetModulePositions();
        poseEstimator.resetPosition(getYaw(), getSwerveModulePositions(), pose);
        swerveOdometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
    }

    public void resetEstimatedHeading(Rotation2d rotation) {
        poseEstimator.resetPosition(getYaw(), getSwerveModulePositions(), new Pose2d(getEstimatedPose().getTranslation(), rotation));
    }

    public void resetOdometryHeading(Rotation2d rotation) {
        poseEstimator.resetPosition(getYaw(), getSwerveModulePositions(), new Pose2d(getOdometryPose().getTranslation(), rotation));
    }

    public void resetOdometryPose(Pose2d pose){
        swerveOdometry.resetPosition(getYaw(), getSwerveModulePositions(), pose);
    }

    public Command resetOdometryToEstimated(){
        return Commands.runOnce(() -> resetOdometryPose(poseEstimator.getEstimatedPosition()));
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public void resetYaw(Rotation2d rotation) {
        gyro.setYaw(rotation.getDegrees());
    }

    public void stop() {
        for(SwerveModule module : swerveModules) {
            module.stopAll();
        }
    }

    @SuppressWarnings("deprecation")
    public void stopOdometryThread(){
        odometryThread.stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIDRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIDRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getSwerveModulePositions());
        field.setRobotPose(getOdometryPose());


        for(SwerveModule module : swerveModules) {
            SmartDashboard.putNumber("Module Drive Position " + module.moduleNumber, module.getModulePosition(true).distanceMeters);
            SmartDashboard.putNumber("Module Steer Position " + + module.moduleNumber, module.getModuleState(true).angle.getDegrees());
        }

        SmartDashboard.putNumber("Pigieon Value Yaw", getYaw().getDegrees());
        SmartDashboard.putString("Mode", "" + driveMode);
        SmartDashboard.putData("Field", field);
    }

    private class OdometryThread extends Thread {
        private BaseStatusSignal[] allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;
        public int ModuleCount = swerveModules.length;

        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryThread() {
            super();
            // 4 signals for each module + 2 for Pigeon2
            allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];

            for (int i = 0; i < ModuleCount; ++i) {
                var signals = swerveModules[i].getSignals();
                allSignals[(i * 4) + 0] = signals[0];
                allSignals[(i * 4) + 1] = signals[1];
                allSignals[(i * 4) + 2] = signals[2];
                allSignals[(i * 4) + 3] = signals[3];
            }
            allSignals[allSignals.length - 2] = gyro.getYaw();
            allSignals[allSignals.length - 1] = gyro.getAngularVelocityZDevice();
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (var sig : allSignals) {
                sig.setUpdateFrequency(250);
            }
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                var status = BaseStatusSignal.waitForAll(0.1, allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);

                /* Get status of the waitForAll */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                for (int i = 0; i < ModuleCount; ++i) {
                    /* No need to refresh since it's automatically refreshed from the waitForAll() */
                    modulePositions[i] = swerveModules[i].getModulePosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(gyro.getYaw(), gyro.getAngularVelocityZDevice()).in(Degrees);

                swerveOdometry.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
            }
        }
        
        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDaqs() {
            return SuccessfulDaqs;
        }

        public int getFailedDaqs() {
            return FailedDaqs;
        }
    }
}
