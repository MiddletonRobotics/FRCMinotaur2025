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

    public class ElevatorConstants {
        public static class ElevatorSpecs {
            public static final double gearing = 6.0;
            public static final double carriageMassKg = 12;
            public static final double drumRadiusMeters = Units.inchesToMeters(2);
            public static final double minHeightMeters = Units.inchesToMeters(0);
            public static final double maxHeightMeters = Units.feetToMeters(6); // remeasure maxV and A
            // public static final boolean simulateGravity = true;
            public static final double startingHeightMeters = 0;
    
            public static final double baseHeight = Units.feetToMeters(3.25);
    
            public static int[] motorIds = { 15, 16 };
            public static boolean[] motorInverted = { false, true };
    
    
    
            public static int zeroOffset = 0;
        }
    
        public static class ElevatorControl {
            public static final double kP = 0.02;
            public static final double kI = 0.0;
            public static final double kD = 0;
            public static final double kS = 0;
            public static final double kG = 2.2977;
            public static final double kV = 2.35; // 12 - 2.3 / 4.139
            public static final double kA = 0;
            public static final double maxV = 4.139;
            public static final double maxA = 3.988; // change in velocity / seconds
        }
    
        public enum ElevatorStates {
            STOP,
            L1,
            L2,
            L3,
            L4,
            MAX,
            STOW
        }
    
        public static class StateHeights {
            public static final double l1Height = Units.inchesToMeters(18);
            public static final double l2Height = Units.inchesToMeters(31.875 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
            public static final double l3Height = Units.inchesToMeters(47.625 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
            public static final double l4Height = Units.inchesToMeters(72 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
        }
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
            public static final int swerveEncoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steeringMotorID, swerveEncoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 4;
            public static final int steeringMotorID = 5;
            public static final int swerveEncoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.143799);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steeringMotorID, swerveEncoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 6;
            public static final int steeringMotorID = 7;
            public static final int swerveEncoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.05);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steeringMotorID, swerveEncoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 8;
            public static final int steeringMotorID = 9;
            public static final int swerveEncoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.3);
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
