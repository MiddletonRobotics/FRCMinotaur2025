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

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utilities.SwerveModuleGearing;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;

// All of the constants that are accessed by other files, to prevent repetition and allows easy changing

public class Constants {
    public static final boolean tuningMode = false;

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

        public static final AngularVelocity LimitedVelocity = DegreesPerSecond.of(120);
        public static final AngularAcceleration LimitedAcceleration = DegreesPerSecondPerSecond.of(120);

        public static final Angle MaximumAllowedPositionError = Degrees.of(2.5);
        public static final AngularVelocity MaximumAllowedVelocityError = DegreesPerSecond.of(2.5);

        public static final Voltage kG = Volts.of(0.35);
        public static final Voltage kS = Volts.of(0.25);
        public static final double kV = 1.14;

        public static final double PositionConversionFactor = (360 / (25 * (42 / 18)) * Math.PI) / 180;
        public static final double VelocityConversionFactor = PositionConversionFactor / 60;
    }

    public static final class ProcessorConstants {
        public static final int pivotMotorID = 18;
        public static final int rollerMotorID = 17;

        public static final AngularVelocity LimitedVelocity = DegreesPerSecond.of(60);
        public static final AngularAcceleration LimitedAcceleration = DegreesPerSecondPerSecond.of(45.0);

        public static final Angle MaximumAllowedPositionError = Degrees.of(3.0);
        public static final AngularVelocity MaximumAllowedVelocityError = DegreesPerSecond.of(3.0);

        public static final Voltage kG = Volts.of(0.425); // 0.425
        public static final Voltage kS = Volts.of(0.195); // 0.195
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
        public static final double ElevatorPositionConversionFactor = ((SprocketPitchDiameter * Math.PI) / ElevatorGearRatio) * 3;
        public static final double ElevatorVelocityConversionFactor = ElevatorPositionConversionFactor / 60;

        /* Elevator PID Characterization Values */
        public static final double ElevatorProfileKp = 0.1;
        public static final double ElevatorProfileKi = 0.00;
        public static final double ElevatorProfileKd = 0.00;

        public static final double ElevatorSparkKp = 0.2;
        public static final double ElevatorSparkKi = 0.01;
        public static final double ElevatorSparkKd = 0.00;

        /* Elevator Motor Characterization Values */
        public static final double ElevatorFeedforwardkS = 0.8;
        public static final double ElevatorFeedforwardkG = 1.1;
        public static final double ElevatorFeedforwardkV = 7.23;
        public static final double ElevatorFeedforwardkA = 0.05;

        public static final LinearVelocity LimitedMaxVelocity = MetersPerSecond.of(5.0);
        public static final LinearAcceleration LimitedMacAcceleration = MetersPerSecondPerSecond.of(5.0);

        public enum ElevatorStates {
            STOP(0),
            L1(0.13124986979166664), // 0.1213 + 0.051
            L2(0.2713302083333333), // 0.2485 + 0.051
            L3(0.5421147135416666), // 0.5099 + 0.051
            L4(0.9606359375 + 0.03), //0.9513 + 0.051 + 0.0254
            BARGE(1.06),
            DEALGEAFIER_L2(0.25148645833333333), // 0.2047 + 0.051
            DEALGEAFIER_L3(0.5184014322916666), // 0.4453 + 0.051
            STOW(0);

            private final double position;

            private ElevatorStates(double position) {
                this.position = position;
            }

            public double getPosition() {
                return this.position;
            }
        }
    }

    public class LimelightConstants {
        public static enum REEFS {
            LEFT, RIGHT
        }

        public static final List<Pose2d> STATION_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(1.151, 1.03, Rotation2d.fromDegrees(55)), // 12 Station
                new Pose2d(1.1383, 7.01, Rotation2d.fromDegrees(-55)) // 13 Station 1.0873
            )
        );

        public static final List<Pose2d> LEFT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(3.735, 3.14, Rotation2d.fromDegrees(60)), // 17 Left
                new Pose2d(3.30, 4.15, Rotation2d.fromDegrees(0)), // 18 Left
                new Pose2d(4.06, 5.105, Rotation2d.fromDegrees(300)), // 19 Left
                new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)), // 20 Left
                new Pose2d(5.70, 3.85, Rotation2d.fromDegrees(180)), // 21 Left
                new Pose2d(4.9113, 2.93927, Rotation2d.fromDegrees(120)) // 22 Left
            )
        );

        public static final List<Pose2d> RIGHT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
                new Pose2d(3.30, 3.85, Rotation2d.fromDegrees(0)), // 18 Right
                new Pose2d(3.713, 4.925, Rotation2d.fromDegrees(300)), // 19 Right
                new Pose2d(4.9489, 5.16, Rotation2d.fromDegrees(240)), // 20 Right
                new Pose2d(5.70, 4.20, Rotation2d.fromDegrees(180)), // 21 Right
                new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)) // 22 Right
            )
        );

        public static final Pose2d BLUE_PROCESSOR = new Pose2d(5.987542, 0.78, Rotation2d.fromDegrees(90));
        public static final Pose2d RED_PROCESSOR = new Pose2d(17.55 - 5.987542, 8.05 - 0.78, Rotation2d.fromDegrees(180));

        public static final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        public static final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694);
    }

    public static final class DriverConstants {
        public static final int driverControllerPort = 0; 
        public static final int operatorControllerPort = 1; 
        public static final int tesingControllerPort = 2; 

        public static final double kDeadband = 0.1;
        public static final boolean disableHAL = false; 

        public static final BlinkinPattern DEF_PATTERN = BlinkinPattern.LARSON_SCANNER_RED;
    }
}
