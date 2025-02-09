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

        SmartDashboard.putData("Field", field);
        BaseStatusSignal.setUpdateFrequencyForAll(50, gyroYaw);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxVelocity);

        for(SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    public void goStraight(Translation2d translation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SwerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), 0.0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxVelocity);

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
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.PhysicalMaxVelocity);
        setModuleStates(states);
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : swerveModules) {
            states[module.moduleNumber] = module.getActualModuleState();
        }

        return states;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module : swerveModules) {
            positions[module.moduleNumber] = module.getModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.SwerveConstants.SwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxVelocity);

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

        SmartDashboard.putNumber("Pigieon Yaw Value", getYaw().getDegrees());
  }
}
