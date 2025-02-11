package frc.robot.utilities.constants;

import com.pathplanner.lib.config.PIDConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.SwerveModuleGearing;

// All of the constants that are accessed by other files, to prevent repetition and allows easy changing

public class Constants {

    public static final class CoralConstants {
        public static final int CAN_ID = 12; // Choose appropriate CAN ID
    }

    public static final class ElevatorConstants {
            public static final int LEFT_CAN_ID = 10;
            public static final int RIGHT_CAN_ID = 11;
            public static final double INCHES_PER_ROTATION = 1.0;
            public static final double MAX_HEIGHT = 36.0;
            public static final double MIN_HEIGHT = 0.0;
            public static final double MAX_VELOCITY = 36.0;
            public static final double MAX_SPEED = 1.0;
            
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kG = 0.0;


            public static final double POSITION_TOLERANCE = 0.1;
            public static final int CURRENT_LIMIT = 40; // Adjust as needed
            public static final double VOLTAGE_COMPENSATION = 12.0;
            public static final double POSITION_CONVERSION_FACTOR = 1.0; // Adjust based on your encoder setup
            public static final double VELOCITY_CONVERSION_FACTOR = 1.0;
        }

    public static final class ModuleConstants {
        /* Swerve Voltage Compensation */
        public static final double voltageCompensation = 12.0;
    
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20; 
        public static final int driveContinuousCurrentLimit = 40; 

        /* PID Values for the Motors. Used to correct the error when trying to move the motors to a desired location */
        public static final double steeringKp = 0.02; 
        public static final double steeringKd = 0.1; 

        public static final double driveKp = 0.1; 
        public static final double driveKd = 0.1; 

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 2;
            public static final int steeringMotorID = 3;
            public static final int swerveEncoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.374267578125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steeringMotorID, swerveEncoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 4;
            public static final int steeringMotorID = 5;
            public static final int swerveEncoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.552734375);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steeringMotorID, swerveEncoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 8;
            public static final int steeringMotorID = 9;
            public static final int swerveEncoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.2373046875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steeringMotorID, swerveEncoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 6;
            public static final int steeringMotorID = 7;
            public static final int swerveEncoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.294189453125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steeringMotorID, swerveEncoderID, angleOffset);
        }
    }

    public static final class SwerveConstants {
        public static final double TrackWidthX = Units.inchesToMeters(28); 
        public static final double WheelBaseY = Units.inchesToMeters(28); 
        public static final double DriveWheelRadius = Math.hypot(WheelBaseY / 2, TrackWidthX / 2);
        public static final double WheelDiameter = Units.inchesToMeters(4.0); 
        public static final double WheelCircumference = WheelDiameter * Math.PI; 

        public static final SwerveModuleGearing SwerveModuleGearRatio = SwerveModuleGearing.MK4N_L2;
        public static final double DriveGearRatio = SwerveModuleGearRatio.getDriveReduction(); 
        public static final double SteerGearRatio = SwerveModuleGearRatio.getSteerReduction(); 

        /* Swerve Kinematics generated by defining the locations of the modules from the center of the robot (if wrong movement by translation will still work, but the rotation will be messed up) */
        public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(TrackWidthX / 2, WheelBaseY / 2), // Divide TrackWidth and WheelBase to define where the module is (+, +)
            new Translation2d(TrackWidthX / 2, -WheelBaseY / 2), // Divide TrackWidth and WheelBase to define where the module is (+, -)
            new Translation2d(-TrackWidthX / 2, WheelBaseY / 2), // Divide TrackWidth and WheelBase to define where the module is (-, +)
            new Translation2d(-TrackWidthX / 2, -WheelBaseY / 2) // Divide TrackWidth and WheelBase to define where the module is (-, -)
        );

        /* Drive Motor Conversion Factors */
        public static final double DriveConversionPositionFactor = (WheelDiameter * Math.PI) / DriveGearRatio;
        public static final double DriveConversionVelocityFactor = DriveConversionPositionFactor / 60.0;
        public static final double SteeringConversionFactor = 360.0 / SteerGearRatio;

        /* Swerve Profiling Values */
        public static final double PhysicalMaxVelocity = Units.feetToMeters(16.5); 
        public static final double PhysicalMaxAcceleration =  Units.feetToMeters(9.5); // 
        public static final double PhysicalMaxAngularVelocity = 2 * Math.PI; 
        public static final double PhysicalMaxAngularAcceleration = 2 * Math.PI;

        /* Motor and Encoder Inversions, they should all br running in CCW+ (either apply positive power when turned counterclockwise or returning postive values when spun counterclockwise) */
        public static final boolean driveInvert = SwerveModuleGearRatio.isDriveInverted();
        public static final boolean angleInvert = SwerveModuleGearRatio.isSteerInverted(); 
        public static final boolean swerveEncoderInverted = false;
        public static final boolean gyroInverted = false;
    }

    public static final class AutonomousConstants {
        public static final double LimitedMaxVelocity = Units.feetToMeters(16.5); 
        public static final double LimitedMaxAcceleration =  Units.feetToMeters(9.5); 
        public static final double LimitedMaxAngularVelocity = 2 * Math.PI; 
        public static final double LimitedlMaxAngularAcceleration = 2 * Math.PI;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(LimitedMaxAngularVelocity, LimitedlMaxAngularAcceleration);

        public static final PIDConstants TranslationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants RotationalPID = new PIDConstants(9, 5.0, 0.0);
    }

    public static final class DriverConstants {
        /* Default ports for our two Controllers */
        public static final int driverControllerPort = 0; 
        public static final int operatorControllerPort = 1; 

        public static final double kDeadband = 0.1;
        public static final boolean disableHAL = false; 

        public static final boolean IS_ALLIANCE_RED = true;
        public static final boolean IS_ALLIANCE_BLUE = !IS_ALLIANCE_RED;
    }
}
