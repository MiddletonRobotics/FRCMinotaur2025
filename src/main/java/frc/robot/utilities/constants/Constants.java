package frc.robot.utilities.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import java.util.Map;

import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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

    public static final class DealgeafierConstants {
        public static final int pivotingMotorID = 19;
        public static final int rollerMotorID = 20;

        public static final AngularVelocity LimitedVelocity = DegreesPerSecond.of(80);
        public static final AngularAcceleration LimitedAcceleration = DegreesPerSecondPerSecond.of(70.0);

        public static final Angle MaximumAllowedPositionError = Degrees.of(2.5);
        public static final AngularVelocity MaximumAllowedVelocityError = DegreesPerSecond.of(2.5);

        public static final Voltage kG = Volts.of(0.0);
        public static final Voltage kS = Volts.of(0.0);
        public static final double kV = 1.14;

        public static final double PositionConversionFactor = 360 / (25 * (42 / 18));
        public static final double VelocityConversionFactor = PositionConversionFactor / 60;
    }

    public static final class ProcessorConstants {
        public static final int pivotMotorID = 18;
        public static final int rollerMotorID = 17;

        public static final AngularVelocity LimitedVelocity = DegreesPerSecond.of(60);
        public static final AngularAcceleration LimitedAcceleration = DegreesPerSecondPerSecond.of(45.0);

        public static final Angle MaximumAllowedPositionError = Degrees.of(3.0);
        public static final AngularVelocity MaximumAllowedVelocityError = DegreesPerSecond.of(3.0);

        public static final Voltage kG = Volts.of(0.478);
        public static final Voltage kS = Volts.of(0.35);
        public static final double kV = 0.48;

        public static final double PositionConversionFactor = (360 / (25 * (42 / 36)) * Math.PI) / 180;
        public static final double VelocityConversionFactor = PositionConversionFactor / 60;
    }

    public class ElevatorConstants {
        public static final int leftElevatorID = 16;
        public static final int rightElevatorID = 15;

        public static final boolean rightElevatorInverted = false;
        public static final boolean leftElevatorInverted = true;

        public static final int ElevatorContinousCurrent = 30;
        public static final double ElevatorVoltageCompensation = 12.0;

        public static final Distance MinimumHeightMeters = Meters.of(0.0);
        public static final Distance MaximumHeightMeters = Feet.of(6);
        public static final Distance StartingHeightMeters = Meters.of(0.0);
        public static final Distance BaseHeight = Feet.of(3.25);

        public static final double ElevatorGearRatio = (60.0 / 10.0);
        public static final double SprocketPitchDiameter = Units.inchesToMeters(((16.0 * 0.25) / Math.PI));
        public static final double ElevatorPositionConversionFactor = SprocketPitchDiameter * Math.PI * ElevatorGearRatio;
        public static final double ElevatorVelocityConversionFactor = ElevatorPositionConversionFactor / 60;

        /* Elevator PID Characterization Values */
        public static final double ElevatorProfileKp = 0.1;
        public static final double ElevatorProfileKi = 0.00;
        public static final double ElevatorProfileKd = 0.00;

        public static final double ElevatorSparkKp = 0.1;
        public static final double ElevatorSparkKi = 0.01;
        public static final double ElevatorSparkKd = 0.00;

        /* Elevator Motor Characterization Values */
        public static final double ElevatorFeedforwardkS = 0.0;
        public static final double ElevatorFeedforwardkG = 2.2977;
        public static final double ElevatorFeedforwardkV = 7.23;
        public static final double ElevatorFeedforwardkA = 0.05;

        public static final LinearVelocity LimitedMaxVelocity = MetersPerSecond.of(1);
        public static final LinearAcceleration LimitedMacAcceleration = MetersPerSecondPerSecond.of(1.25);

        public static class ElevatorHeights {
            public static final double l1Height = 2.0465;
            public static final double l2Height = 4.9203;
            public static final double l3Height = 9.5794;
            public static final double l4Height = 11.00;
            public static final double barge = Units.inchesToMeters(0);
            public static final double l2Dealgeafier = Units.inchesToMeters(0);
            public static final double l3Dealgeafier = Units.inchesToMeters(0);
            public static final double stow = Units.inchesToMeters(0);
        }

        public enum ElevatorStates {
            STOP,
            L1,
            L2,
            L3,
            L4,
            BARGE,
            DEALGEAFIER_L2,
            DEALGEAFIER_L3,
            STOW
        }
    }

    public static class ElevatorConstants2 {
        // MOTOR CAN BUS IDS
        public static final int kLeaderID = 46;
        public static final int kFollowerID = 45;
        // RAW PID CONSTANTS TODO: TUNE
        public static final double kP = .1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        // MAXMOTION CONSTANTS TODO: TUNE
        public static final double kMAXMotionP = 0.1;
        public static final double kMAXMotionI = 0;
        public static final double kMAXMotionD = 0;
        public static final double kMAXMotionFF = 0;
        public static final double kMAXMotionMaxAcceleration = 1.25;
        public static final double kMAXMotionMaxVelocity = 1;
        public static final double kMAXMotionAllowedError = 0.01;
        // TOLERANCE FOR PID ERROR
        public static final double kTolerance = 1.0; // TODO: TUNE
        // LIMIT VALUES
        public static final double kMinimumRotationLimit = -5; // TODO: SET
        public static final double kMaximumRotationLimit = 100; // TODO: SET
        public static final double kMinimumOutputLimit = -.8;
        public static final double kMaximumOutputLimit = .8;
        // INVERSIONS
        public static final boolean kInverted = false;
        public static final boolean kFollowerInverted = true;
        // CURRENT LIMITS TODO: TUNE
        public static final int kStallLimit = 40;
        public static final int kFreeLimit = 40;
        // IDLE MODE
        public static final IdleMode kIdleMode = IdleMode.kBrake;
    }

    public class LimelightConstants {
        public static final Distance leftLimelightForward = Inches.of(7.415);
        public static final Distance leftLimelightLateral = Inches.of(11.613);
        public static final Distance leftLimelightHeight = Inches.of(5.872);
        public static final Angle leftLimelightRoll = Degrees.of(0);
        public static final Angle leftLimelightPitch = Degrees.of(20.0);
        public static final Angle leftLimelightYaw = Degrees.of(330.0);

        public static final Distance rightLimelightForward = Inches.of(7.214);
        public static final Distance rightLimelightLateral = Inches.of(11.320);
        public static final Distance rightLimelightHeight = Inches.of(5.721);
        public static final Angle rightLimelightRoll = Degrees.of(0);
        public static final Angle rightLimelightPitch = Degrees.of(20.0);
        public static final Angle rightLimelightYaw = Degrees.of(30.0);
    }

    public static final class DriverConstants {
        public static final int driverControllerPort = 0; 
        public static final int operatorControllerPort = 1; 
        public static final int tesingControllerPort = 2; 

        public static final double kDeadband = 0.1;
        public static final boolean disableHAL = false; 
    }
}
