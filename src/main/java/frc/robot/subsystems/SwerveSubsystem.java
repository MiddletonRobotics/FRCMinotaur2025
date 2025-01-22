//Import required packages to apply swerve drive to robot.
package frc.robot.subsystems;

import java.security.cert.X509CRL;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.constants.Constants;

/* Sets up class that assigns motors to each swerve module and get swerving.
* Methods created to handle different actions taken on the controls.
*/
public class SwerveSubsystem extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] swerveModules;

    private RobotConfig robotConfiguration;

    private BaseStatusSignal gyroYaw;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

    private Field2d field;

    public SwerveSubsystem() {
        gyro = new Pigeon2(13);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);

        gyroYaw = gyro.getYaw();

        zeroHeading();
        
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.ModuleConstants.FrontLeftModule.constants),
            new SwerveModule(1,Constants.ModuleConstants.FrontRightModule.constants),
            new SwerveModule(2,Constants.ModuleConstants.BackLeftModule.constants),
            new SwerveModule(3,Constants.ModuleConstants.BackRightModule.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.SwerveKinematics, getYaw(), getSwerveModulePositions());
        field = new Field2d();

        /*

        try{
            robotConfiguration = RobotConfig.fromGUISettings();
          } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
          }

        AutoBuilder.configure(
            this::getPose, 
            this::resetSwerveOdometry,
            this::getRobotRelativeSpeeds, 
            this::driveRobotRelative, 
            new PPHolonomicDriveController(
                Constants.AutonomousConstants.TranslationPID, 
                Constants.AutonomousConstants.RotationalPID
            ), 
            robotConfiguration,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        */

        SmartDashboard.putData("Field", field);
        BaseStatusSignal.setUpdateFrequencyForAll(50, gyroYaw);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
        }
    }

    public void goStraight(Translation2d translation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), 0.0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for (SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Command to drive the robot using robot relative speeds
     * @param speeds The ChassisSpeeds that contains the movement as vx, vy, and omega
     */

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states =  Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);
        setModuleStates(states);
    }

    /**
    * Command to go to a pose using a Trapezoidal PID profile for increased accuracy compared to a pure on the fly
    * @param targetPose the Supplier<Pose2d> that the robot should drive to
    * @return command to PID align to a pose on the field
    */

    public Command chasePoseCommand(Supplier<Pose2d> target){
        TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);
        
        ProfiledPIDController xController = new ProfiledPIDController(0.1, 0, 0, X_CONSTRAINTS);
        ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0, Y_CONSTRAINTS);
        ProfiledPIDController omegaController = new ProfiledPIDController(0.01, 0, 0, OMEGA_CONSTRAINTS);

        xController.setTolerance(0.3);
        yController.setTolerance(0.3);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-180, 180);

        return new DeferredCommand(() ->
            new RepeatCommand(
                new FunctionalCommand(
                () -> {
                    // Init
                },
                () -> {
                    double xSpeed = xController.calculate(this.getPose().getX(), target.get().getX());
                    double ySpeed = yController.calculate(this.getPose().getY(), target.get().getY());
                    double omegaSpeed = omegaController.calculate(this.getYaw().getDegrees(), target.get().getRotation().getDegrees());

                    double xSpeedDelivered = translationLimiter.calculate(xSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                    double ySpeedDelivered = strafeLimiter.calculate(ySpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                    double rotDelivered = rotationLimiter.calculate(omegaSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);

                    drive(new Translation2d(xSpeedDelivered, ySpeedDelivered).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond), rotDelivered * Constants.SwerveConstants.PhysicalAngularMaxVelocity, false, true);
                },
                interrupted -> {
                    this.drive(new Translation2d(0, 0),0, true, true);
                    System.out.println("30 cm away now");
                },
                () -> {
                    return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
                },
                this)
            ), Set.of(this)
        );
    }

    /**
   * Command to go to a pose using a Trapezoidal PID profile for increased accuracy compared to a pure on the fly
   * @param targetPose the Supplier<Pose2d> that the robot should drive to ROBOT RELATIVE
   * @return command to PID align to a pose that is ROBOT RELATIVE
   */

    public Command chasePoseRobotRelativeCommand(Supplier<Pose2d> target){
        TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        //TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);
        
        ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0, X_CONSTRAINTS);
        ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0, Y_CONSTRAINTS);
        PIDController omegaPID = new PIDController(0.01, 0, 0);

        xController.setTolerance(0.10);
        yController.setTolerance(0.03);
        omegaPID.setTolerance(1.5);
        omegaPID.enableContinuousInput(-180, 180);

        return new DeferredCommand(() ->
            new FunctionalCommand(
            () -> {
                // Init
            },
            () -> {
                double xSpeed = xController.calculate(0, target.get().getX());
                double ySpeed = yController.calculate(0, target.get().getY());
                double omegaSpeed = omegaPID.calculate(0, target.get().getRotation().getDegrees());

                double xSpeedDelivered = translationLimiter.calculate(xSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                double ySpeedDelivered = strafeLimiter.calculate(ySpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);
                double rotDelivered = rotationLimiter.calculate(omegaSpeed) * (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.3);

                drive(new Translation2d(xSpeedDelivered, ySpeedDelivered).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond), rotDelivered * Constants.SwerveConstants.PhysicalAngularMaxVelocity, true, true);
            },

            interrupted -> {
                this.drive(new Translation2d(0, 0),0, false, true);
                omegaPID.close();
                System.out.println("Aligned now");
            },

            () -> {
            return omegaPID.atSetpoint() && xController.atGoal() && yController.atGoal();
            },
            this), Set.of(this)
        );
    }

    /**
    * A method to travel to a given position on the field given your current location on the field.
    * @param targetPose a {@link Pose2d} representing the pose to travel to.
    * @return a Command that continuously attempts to converge to targetPose until the controllers' thresholds have been reached.
    */

    public Command goToPoint(Pose2d targetPose) {
        PIDController xController = new PIDController(Constants.ModuleConstants.driveKP, Constants.ModuleConstants.driveKI, Constants.ModuleConstants.driveKD);
        PIDController yController = new PIDController(Constants.ModuleConstants.driveKP, Constants.ModuleConstants.driveKI, Constants.ModuleConstants.driveKD);
        PIDController thetaController = new PIDController(Constants.ModuleConstants.angleKP, Constants.ModuleConstants.angleKI, Constants.ModuleConstants.angleKD);

        return new FunctionalCommand(
            () ->
                System.out.println(String.format("Traveling to x:%s, y:%s, z:%s", targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees())),
            () -> {
                double sX = xController.calculate(getPose().getX(), targetPose.getX());
                double sY = yController.calculate(getPose().getY(), targetPose.getY());
                double sR = thetaController.calculate(getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

                double xSpeedDelivered = translationLimiter.calculate(sX) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
                double ySpeedDelivered = strafeLimiter.calculate(sY) * Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
                double rotDelivered = rotationLimiter.calculate(sR) * Constants.SwerveConstants.PhysicalAngularMaxVelocity;

                drive(new Translation2d(xSpeedDelivered, ySpeedDelivered).times(Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond), rotDelivered * Constants.SwerveConstants.PhysicalAngularMaxVelocity, false, true);
            },
            interrupted -> {
                xController.close();
                yController.close();
                thetaController.close();
            },
            () -> xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint(),
            this
        );
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : swerveModules) {
            states[module.moduleNumber] = module.getSwerveModuleState();
        }

        return states;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module : swerveModules) {
            positions[module.moduleNumber] = module.getSwerveModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.SwerveConstants.SwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }


    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(new Rotation2d(), getSwerveModulePositions(), pose);
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public Rotation2d getYaw() {
        return (Constants.SwerveConstants.gyroInverted) ? Rotation2d.fromDegrees(360 - gyroYaw.getValueAsDouble()) : Rotation2d.fromDegrees(gyroYaw.getValueAsDouble());
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void zeroHeading() {  
        gyro.setYaw(0.0);
    }

    public void stop() {
        for(SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(gyroYaw);
        swerveOdometry.update(getYaw(), getSwerveModulePositions());
        field.setRobotPose(getPose());

        double measuredStates[] = {
            swerveModules[0].getSwerveModuleState().angle.getDegrees(),
            swerveModules[0].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[1].getSwerveModuleState().angle.getDegrees(),
            swerveModules[1].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[2].getSwerveModuleState().angle.getDegrees(),
            swerveModules[2].getSwerveModuleState().speedMetersPerSecond,
            swerveModules[3].getSwerveModuleState().angle.getDegrees(),
            swerveModules[3].getSwerveModuleState().speedMetersPerSecond,
        };

        double desiredStates[] = {
            swerveModules[0].getDesiredState().angle.getDegrees(),
            swerveModules[0].getDesiredState().speedMetersPerSecond,
            swerveModules[1].getDesiredState().angle.getDegrees(),
            swerveModules[1].getDesiredState().speedMetersPerSecond,
            swerveModules[2].getDesiredState().angle.getDegrees(),
            swerveModules[2].getDesiredState().speedMetersPerSecond,
            swerveModules[3].getDesiredState().angle.getDegrees(),
            swerveModules[3].getDesiredState().speedMetersPerSecond,
        };

        double loggingEncoders[] = {
            swerveModules[0].getSwerveEncoder().getDegrees(),
            swerveModules[1].getSwerveEncoder().getDegrees(),
            swerveModules[2].getSwerveEncoder().getDegrees(),
            swerveModules[3].getSwerveEncoder().getDegrees(),
        };

        double loggingRelEncoder[] = {
            swerveModules[0].getAngle().getDegrees(),
            swerveModules[1].getAngle().getDegrees(),
            swerveModules[2].getAngle().getDegrees(),
            swerveModules[3].getAngle().getDegrees(),
        };


        SmartDashboard.putNumberArray("MeasuredSwerveStates", measuredStates);
        SmartDashboard.putNumberArray("DesiredSwerveStates", desiredStates);

        SmartDashboard.putNumber("Front-Left ABS Encoder Position", loggingEncoders[0]);
        SmartDashboard.putNumber("Front-Right ABS Encoder Position", loggingEncoders[1]);
        SmartDashboard.putNumber("Back-Left ABS Encoder Position", loggingEncoders[2]);
        SmartDashboard.putNumber("Back-Right ABS Encoder Position", loggingEncoders[3]);

        SmartDashboard.putNumber("Front-Left Rel Encoder Position", loggingRelEncoder[0]);
        SmartDashboard.putNumber("Front-Right Rel Encoder Position", loggingRelEncoder[1]);
        SmartDashboard.putNumber("Back-Left Rel Encoder Position", loggingRelEncoder[2]);
        SmartDashboard.putNumber("Back-Right Rel Encoder Position", loggingRelEncoder[3]);

        SmartDashboard.putNumber("Pigieon Yaw Value", getYaw().getDegrees());
  }
}
