package frc.robot.utilities.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.Map;

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
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utilities.SwerveModuleGearing;

// All of the constants that are accessed by other files, to prevent repetition and allows easy changing

public class Constants {
    public static final boolean debug = true;

    public static final class ModuleConstants {
        public static final class SparkMaxModuleConstants {
            public static final double VoltageCompensation = 12.0;

            public static final int SteerSmartCurrentLimit = 20; 
            public static final int SteerSecondaryCurrentLimit = 50;

            public static final int DriveSmartCurrentLimit = 40; 
            public static final int DriveSecondaryCurrentLimit = 70;

            public static final double steeringKp = 0.02; 
            public static final double steeringKd = 0.1; 

            public static final double driveKp = 0.1; 
            public static final double driveKd = 0.1; 

            public static final double driveKS = 0.667;
            public static final double driveKV = 2.44;
            public static final double driveKA = 0.27;
        }

        public static final class TalonFXModuleConstants {
            public static final double driveKp = 10.0; 
            public static final double driveKd = 0.0; 

            public static final double driveKS = 0.667;
            public static final double driveKV = 2.44;
            public static final double driveKA = 0.27;

            public static final double steeringKp = 10.0; 
            public static final double steeringKd = 0.0; 

            public static final boolean driveStatorCurrentLimitEnable = true;
            public static final double driveStatorCurrentLimit = 80;

            public static final boolean steerStatorCurrentLimitEnable = true;
            public static final double steerStatorCurrentLimit = 40;
        }

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
        public static final double PhysicalMaxAcceleration =  Units.feetToMeters(8.0); 
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

        public static final PIDConstants TranslationPID = new PIDConstants(10.0, 0.0, 0.0);
        public static final PIDConstants RotationalPID = new PIDConstants(10.0, 5.0, 0.0);
    }


    public static final class AlgeaProcessorConstants {
        public static final int rollerID = 17;
        public static final int pivotingID = 18;
    }
    public static final class CoralSpinnerConstants {
        //public static final int CAN_ID = 21;// Choose appropriate CAN ID
    }
     
    public static final class CoralPivotConstants {
        public static final int CAN_ID = 55;
    }

    public static final class AlgeaSpinnerConstants {
        public static final int CAN_ID = 46;// Choose appropriate CAN ID
    }

    public class ElevatorConstants {
        public static final int leftElevatorID = 16;
        public static final int rightElevatorID = 15;

        public static final boolean rightElevatorInverted = false;
        public static final boolean leftElevatorInverted = true;

        public static final int ElevatorContinousCurrent = 30;
        public static final double ElevatorVoltageCompensation = 12.0;

        public static final double MinimumHeightMeters = Units.inchesToMeters(0);
        public static final double MaximumHeightMeters = Units.feetToMeters(6);
        public static final double StartingHeightMeters = 0;
        public static final double BaseHeight = Units.feetToMeters(3.25);

        public static final double ElevatorGearRatio = (60.0 / 10.0);
        public static final double SprocketPitchDiameter = Units.inchesToMeters(((16.0 * 0.25) / Math.PI));
        public static final double ElevatorPositionConversionFactor = SprocketPitchDiameter * Math.PI / ElevatorGearRatio;
        public static final double ElevatorVelocityConversionFactor = SprocketPitchDiameter * Math.PI / 60 * ElevatorGearRatio;

        /* Elevator PID Characterization Values */
        public static final double ElevatorProfileKp = 0.01;
        public static final double ElevatorProfileKi = 0.00;
        public static final double ElevatorProfileKd = 0.00;

        public static final double ElevatorSparkKp = 0.03;
        public static final double ElevatorSparkKi = 0.01;
        public static final double ElevatorSparkKd = 0.00;

        /* Elevator Motor Characterization Values */
        public static final double ElevatorFeedforwardkS = 0.0;
        public static final double ElevatorFeedforwardkG = 2.2977;
        public static final double ElevatorFeedforwardkV = 7.23;
        public static final double ElevatorFeedforwardkA = 0.05;

        public static final LinearVelocity LimitedMaxVelocity = MetersPerSecond.of(4.139);
        public static final LinearAcceleration LimitedMacAcceleration = MetersPerSecondPerSecond.of(3.988);

        public static class ElevatorHeights {
            public static final double l1Height = Units.inchesToMeters(18);
            public static final double l2Height = Units.inchesToMeters(31.875 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
            public static final double l3Height = Units.inchesToMeters(47.625 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
            public static final double l4Height = Units.inchesToMeters(72 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
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
    }

    public class LimelightConstants {

        
        public static final double sourceZoneAprilTagHeight = 53.38;
        public static final double speakerAprilTagHeight = 57.13;
        public static final double ampZoneAprilTagHeight = 53.38;
        public static final double stageAprilTagHeight = 52.00;
        public static final Map<Integer, Double> tagPipelinesHeights = Map.ofEntries(
            Map.entry(1, sourceZoneAprilTagHeight),
            Map.entry(2, sourceZoneAprilTagHeight),
            Map.entry(3, speakerAprilTagHeight),
            Map.entry(4, speakerAprilTagHeight),
            Map.entry(5, ampZoneAprilTagHeight),
            Map.entry(6, ampZoneAprilTagHeight),
            Map.entry(7, speakerAprilTagHeight),
            Map.entry(8, speakerAprilTagHeight),
            Map.entry(9, sourceZoneAprilTagHeight),
            Map.entry(10, sourceZoneAprilTagHeight),
            Map.entry(11, stageAprilTagHeight),
            Map.entry(12, stageAprilTagHeight),
            Map.entry(13, stageAprilTagHeight),
            Map.entry(14, stageAprilTagHeight),
            Map.entry(15, stageAprilTagHeight),
            Map.entry(16, stageAprilTagHeight)
        );
    }

    public static final class DriverConstants {
        public static final int driverControllerPort = 0; 
        public static final int operatorControllerPort = 1; 
        public static final int tesingControllerPort = 2; 

        public static final double kDeadband = 0.1;
        public static final boolean disableHAL = false; 
    }
}
