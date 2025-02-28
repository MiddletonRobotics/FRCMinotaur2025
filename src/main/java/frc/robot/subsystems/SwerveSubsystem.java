package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utilities.constants.Constants;

/* Sets up class that assigns motors to each swerve module and get swerving.
* Methods created to handle different actions taken on the controls.
*/
public class SwerveSubsystem extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModuleSpark[] swerveModules;

    private RobotConfig robotConfiguration;
    private BaseStatusSignal gyroYaw;

    private Field2d field;

    //private final SysIdRoutine sysIDRoutine;
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public enum DriveMode {
        FIELD_RELATIVE,
        ROBOT_RELATIVE
    }

    private DriveMode driveMode;

    public SwerveSubsystem() {
        System.out.println("[Initialization] Creating SwerveSubsystem");

        gyro = new Pigeon2(14);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);

        gyroYaw = gyro.getYaw();

        zeroHeading();
        
        swerveModules = new SwerveModuleSpark[] {
            new SwerveModuleSpark(0, Constants.ModuleConstants.FrontLeftModule.constants),
            new SwerveModuleSpark(1, Constants.ModuleConstants.FrontRightModule.constants),
            new SwerveModuleSpark(2, Constants.ModuleConstants.BackLeftModule.constants),
            new SwerveModuleSpark(3, Constants.ModuleConstants.BackRightModule.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.SwerveKinematics, getYaw(), getSwerveModulePositions());
        field = new Field2d();

        try {
            robotConfiguration = RobotConfig.fromGUISettings();
          } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, 
            this::resetSwerveOdometry,
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
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

        driveMode = DriveMode.FIELD_RELATIVE;

        /*

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
                    .linearPosition(m_distance.mut_replace(swerveModules[0].getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[0].getDriveVelocity(), MetersPerSecond));
                log.motor("Front-Right Module")
                    .voltage(m_appliedVoltage.mut_replace(swerveModules[1].getDriveMotor() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(swerveModules[1].getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[0].getDriveVelocity(), MetersPerSecond));

                log.motor("Back-Left Module")
                    .voltage(m_appliedVoltage.mut_replace(swerveModules[2].getDriveMotor() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(swerveModules[2].getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[0].getDriveVelocity(), MetersPerSecond));

                log.motor("Back-Right Module")
                    .voltage(m_appliedVoltage.mut_replace(swerveModules[3].getDriveMotor() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(swerveModules[3].getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(swerveModules[0].getDriveVelocity(), MetersPerSecond));
                }, 
            this)
        );

        */

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        BaseStatusSignal.setUpdateFrequencyForAll(50, gyroYaw);
    }

    public void switchDriveMode() {
       if(this.driveMode == DriveMode.FIELD_RELATIVE) {
        this.driveMode = DriveMode.ROBOT_RELATIVE;
       } else if (this.driveMode == DriveMode.ROBOT_RELATIVE) {
        this.driveMode = DriveMode.FIELD_RELATIVE;
       }
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates;

        switch(driveMode) {
            case FIELD_RELATIVE:
                swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()));
                break;
            case ROBOT_RELATIVE:
                swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
                break;
            default:
                swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxVelocity);

        for(SwerveModuleSpark module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    public void goStraight(Translation2d translation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), 0.0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxVelocity);

        for (SwerveModuleSpark module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Command to drive the robot using robot relative speeds
     * @param speeds The ChassisSpeeds that contains the movement as vx, vy, and omega
     */

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states =  Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.PhysicalMaxVelocity);
        setModuleStates(states);
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleSpark module : swerveModules) {
            states[module.moduleNumber] = module.getActualModuleState();
        }

        return states;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModuleSpark module : swerveModules) {
            positions[module.moduleNumber] = module.getModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.SwerveConstants.SwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxVelocity);

        for(SwerveModuleSpark module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    /*
     
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIDRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIDRoutine.dynamic(direction);
    }

    */

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }


    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(new Rotation2d(), getSwerveModulePositions(), pose);
    }

    public void resetModulesToAbsolute() {
        for(SwerveModuleSpark module : swerveModules) {
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
        for(SwerveModuleSpark module : swerveModules) {
            module.stop();
        }
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(gyroYaw);
        swerveOdometry.update(getYaw(), getSwerveModulePositions());
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Pigieon Value Yaw", gyroYaw.getValueAsDouble());
        SmartDashboard.putString("Mode", "" + driveMode);
        SmartDashboard.putData("Field", field);
  }
}
