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

// All of the constants that are accessed by other files, to prevent repetition and allows easy changing

public class Constants {
    public static final class ModuleConstants {
        /* Swerve Voltage Compensation */
        public static final double voltageCompensation = 12.0; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20; // Steering the swerve modules requires less power, and doesn't have a lot of movement, therefore we can reduce the amperes we are feeding it
        public static final int driveContinuousCurrentLimit = 40; // Drive motors should be at the maximum reccomended amperes to get the most power and speed from it

        /* PID Values for the Motors. Used to correct the error when trying to move the motors to a desired location */
        public static final double angleKP = 0.02; // Propotional: If there is error, move the motor propotional to the error
        public static final double angleKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double angleKD = 0.1; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double angleKFF = 0.0; // Force: Additional gain for creating offsts

        public static final double driveKP = 0.1; // Propotional: If there is error, move the motor propotional to the error
        public static final double driveKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double driveKD = 0.1; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double driveKFF = 0.0; // Force: Additional gain for creating offsts

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.374267578125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, false, false, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.552734375);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, false, false, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.2373046875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, false, true, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.294189453125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, false, false, canCoderID, angleOffset);
        }
    }

    public static final class SwerveConstants {
        public static final double TrackWidth = Units.inchesToMeters(28); // Distance from center from the right wheels to the left wheels (must be converted to meters)
        public static final double WheelBase = Units.inchesToMeters(28); // Distance from center from the front wheels to the back wheels (must be converted to meters)
        public static final double WheelDiameter = Units.inchesToMeters(4.0); // Diameter of the wheel attached to the swerve modules (must be converted to meters)
        public static final double WheelCircumference = WheelDiameter * Math.PI; // To get the circumference multiply the diameter by PI

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double DriveGearRatio = (5.9 / 1.0); // Our MK3 modules are configured with the fast gear ratio (6.86:1)
        public static final double AngleGearRatio = (18.75); // Steering ratio on the MK3 modules is 12.8:1

        /* Swerve Kinematics generated by defining the locations of the modules from the center of the robot (if wrong movement by translation will still work, but the rotation will be messed up) */
        public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(WheelBase / 2, TrackWidth / 2), // Divide TrackWidth and WheelBase to define where the module is (+, +)
            new Translation2d(WheelBase / 2, -TrackWidth / 2), // Divide TrackWidth and WheelBase to define where the module is (+, -)
            new Translation2d(-WheelBase / 2, TrackWidth / 2), // Divide TrackWidth and WheelBase to define where the module is (-, +)
            new Translation2d(-WheelBase / 2, -TrackWidth / 2) // Divide TrackWidth and WheelBase to define where the module is (-, -)
        );

        /* Drive Motor Conversion Factors */
        public static final double DriveConversionPositionFactor = (WheelDiameter * Math.PI) / DriveGearRatio;
        public static final double DriveConversionVelocityFactor = DriveConversionPositionFactor / 60.0;
        public static final double AngleConversionFactor = 360.0 / AngleGearRatio;

        /* Swerve Profiling Values */
        public static final double PhysicalMaxSpeedMetersPerSecond = 5.24; // (4.1) Maximum speed in meters per second that the Swerve Modules allow you to go
        public static final double PhysicalMaxAcceleration = 2.5; // (1.9)
        public static final double PhysicalAngularMaxVelocity = 3 * Math.PI; // Maxiumum speed in radians per seconr that the swerve module is able to rotate (6.28 radians per full rotation)
        public static final double PhysicalMaxAngularAcceleration = 4.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake; // What the steering motor should do when not applied with any power (should always be brake while running to prevent overshooting target)
        public static final IdleMode driveNeutralMode = IdleMode.kBrake; // What the drive motor should do when not applied with any power (should always be brake while running to prevent overshooting target)

        public static final IdleMode driveIdleMode = IdleMode.kCoast; // What the drive motor should so when the robot hasn't been initialized
        public static final IdleMode angleIdleMode = IdleMode.kCoast; // What the steering motor should so when the robot hasn't been initialized

        /* Motor and Encoder Inversions, they should all br running in CCW+ (either apply positive power when turned counterclockwise or returning postive values when spun counterclockwise) */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true; 
        public static final boolean swerveEncoderInverted = false;
        public static final boolean gyroInverted = false;
    }

    public static final class AutonomousConstants {
        public static final double PhysicalMaxSpeedMetersPerSecond = 4.4;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double DriveBaseRadius = Math.hypot(SwerveConstants.TrackWidth / 2, SwerveConstants.WheelBase / 2); // 0.503

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);

        public static final PIDConstants TranslationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants RotationalPID = new PIDConstants(9, 5.0, 0.0);
    }

     public static class VisionConstants {
        public static final boolean DRIVER_CAMERA_ACTIVE = false;
        public static final boolean VISION_OVERRIDE_ENABLED = false;
        public static final boolean IS_LIMELIGHT_MODE = false;
        public static final boolean IS_PHOTON_VISION_MODE = true;// TODO: Change to false
        public static final boolean IS_NEURAL_NET = true;
        public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.5; // Max difference between vision and odometry pose estimate
        public static final String DRIVER_CAM_STREAM = "http://drivercam.local:1182/stream.mjpg";    
    
        // Field limits
        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;
        public static final double VISION_X_MIN_CUTOFF = 3.0;
        public static final double VISION_X_MAX_CUTOFF = 13.5;
    
        // Limelight - units are meters
        public static final String POSE_LIMELIGHT = "limelight-pose";
        public static final String NN_LIMELIGHT = "limelight-nn";
        public static final int APRILTAG_PIPELINE = 1;
        public static final int NOTE_DETECTOR_PIPELINE = 2;
    
        public static final double POSE_LIME_X = 0.322; // Forward - Meters
        public static final double POSE_LIME_Y = -0.274; // Side - Right is positive on the limelight
        public static final double POSE_LIME_Z = 0.21; // Up
        public static final double POSE_LIME_PITCH = 30; // NEED to find units - degrees for now
        public static final double POSE_LIME_ROLL = 0.0;
        public static final double POSE_LIME_YAW = 0.0;
    
        //THESE ARE ALL ROBOT RELATIVE (CENTER OF THE ROBOT IS THE ORIGIN)
        public static final double NN_LIME_X = -0.308; // +X is forward on the robot
        public static final double NN_LIME_Y = -0.300; // +Y is the the left of the robot
        public static final double NN_LIME_Z = 0.497; // +Z is up
        public static final double NN_LIME_PITCH = -0.349;
        public static final double NN_LIME_ROLL = 0.0;
        public static final double NN_LIME_YAW = Math.PI;
    
        public static final Transform2d NN_ROBOT_TO_LIME_2D = new Transform2d(NN_LIME_X, NN_LIME_Y, new Rotation2d(NN_LIME_YAW));
        public static final Transform2d NN_LIME_TO_ROBOT_2D = new Transform2d(-NN_LIME_X, -NN_LIME_Y, new Rotation2d(-NN_LIME_YAW));
    
        // Photonvision
        public static final double POSE_AMBIGUITY_CUTOFF = 0.2;
        public static final String POSE_PHOTON_1 = "photoncam-1";

        // Translation Values (location relative to robot center)
        public static final double CAM_1_X = 0.3079; // Forward: camera To Robot XMeters
        public static final double CAM_1_Y = -0.0762; // Left: camera To Robot YMeters
        public static final double CAM_1_Z = 0.156; // Up: camera To Robot ZMeters
    
        // Rotation mounting angles (roll-pitch-yaw) in RADIANS
        public static final double CAM_1_ROLL_RADIANS = 0.0; // camera Roll Radians
        public static final double CAM_1_PITCH_RADIANS = 0.436; // camera Pitch Radians
        public static final double CAM_1_YAW_RADIANS = 0.0; // camera Yaw Radians, +CCW
    
        public static final Transform3d PHOTON_1_ROBOT_TO_CAM = new Transform3d(CAM_1_X, CAM_1_Y, CAM_1_Z, new Rotation3d(CAM_1_ROLL_RADIANS, CAM_1_PITCH_RADIANS, CAM_1_YAW_RADIANS));
        public static final Transform3d PHOTON_1_CAM_TO_ROBOT = new Transform3d(-CAM_1_X, -CAM_1_Y, -CAM_1_Z, new Rotation3d(-CAM_1_ROLL_RADIANS, -CAM_1_PITCH_RADIANS, -CAM_1_YAW_RADIANS)); 
    
        // Photonvision
        public static final String POSE_PHOTON_2 = "photoncam-2";
        // Translation Values (location relative to robot center)
        public static final double CAM_2_X = 0.5; // Forward: camera To Robot XMeters
        public static final double CAM_2_Y = 0.0; // Right: camera To Robot YMeters
        public static final double CAM_2_Z = 0.2; // Up: camera To Robot ZMeters
    
        // Rotation mounting angles (roll-pitch-yaw) in RADIANS
        public static final double CAM_2_ROLL_RADIANS = 0.0; // camera Roll Radians
        public static final double CAM_2_PITCH_RADIANS = 0.2618; // camera Pitch Radians
        public static final double CAM_2_YAW_RADIANS = 0.0; // camera Yaw Radians
    }


    public static class PoseConstants {
        // Increase these numbers to trust your model's state estimates less.
        public static final double kPositionStdDevX = 0.1;
        public static final double kPositionStdDevY = 0.1;
        public static final double kPositionStdDevTheta = 10;
    
        // Increase these numbers to trust global measurements from vision less.
        public static final double kVisionStdDevX = 2.5;
        public static final double kVisionStdDevY = 2.5;
        public static final double kVisionStdDevTheta = 500;
      }

    public static final class IntakeConstants {
        /* Hardware ID of CAN */
        public static final int rollerMotorID = 16;
        public static final int pivotMotorID = 15;
        public static final int pivotEncoderID = 19;

        /* Climber Current Limiting */
        public static final int pivotContinuousCurrentLimit = 60; 
        public static final int rollerContinuousCurrentLimit = 70; 

        /* Encoder Offsets and positions */
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0);
        public static final double deployIntakeSetpoint = 0.0;
        public static final double storeIntakeSetpoint = 0.0;
        public static final double goalSetpointErrorTolerence = 0.3;

        /* Motor and Encoder Inversions */
        public static final boolean rollerMotorInvert = false;
        public static final boolean pivotMotorInvert = true;

        public static final boolean pivotEnocderInverted = false;

        /* Gear Ratios */
        public static final double pivotGearRatio = (20 / 1.0);
        public static final double rollerGearRatio = (1 / 1.0);

        /* Motor Conversion Factors */
        public static final double AngleConversionFactor = 360.0 / pivotGearRatio;

        /* PID Values for the Motors. Used to correct the error when trying to move the motors to a desired location */
        public static final double pivotKP = 0.05; // Propotional: If there is error, move the motor propotional to the error
        public static final double pivotKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double pivotKD = 0.0; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double pivotKFF = 0.0; // Force: Additional gain for creating offsts

        /* PID Values for the Motors. Used to correct the error when trying to move the motors to a desired location */
        public static final double rollerKP = 0.05; // Propotional: If there is error, move the motor propotional to the error
        public static final double rollerKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double rollerKD = 0.0; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double rollerKFF = 0.0; // Force: Additional gain for creating offsts

       /* Neutral Modes */ 
        public static final IdleMode rollerMotorNeutralMode = IdleMode.kBrake; // What the roller motor should do when not applied with any power (should always be brake while running to prevent overshooting target)
        public static final IdleMode pivotMotorNeutralMode = IdleMode.kBrake; // What the pivot motor should do when not applied with any power (should always be brake while running to prevent overshooting target)

        public static final IdleMode rollerMotorIdleMode = IdleMode.kCoast; // What the roller motor should so when the robot hasn't been initialized
        public static final IdleMode pivotMotorIdleMode = IdleMode.kCoast; // What the pivot motor should so when the robot hasn't been initialized

        /* Intake Voltage Compensation */
        public static final double voltageCompensation = 12; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)
    }

    public static final class ShooterConstants {

        /* Hardware ID from CAN */
        public static final int lowerShooterMotorID = 13; // Motor ID of the motor thats rolls the lower wheels on the shooter
        public static final int upperShooterMotorID = 14; // Motor ID of the motor thats rolls the upper wheels on the shooter

        /* Motor Inversions (to where postive values shooter a game piece out, or CW+) */
        public static final boolean lowerShooterMotorInvert = true; 
        public static final boolean upperShooterMotorInvert = false;

        /* Neutral Modes */ 
        public static final IdleMode lowerShooterMotorNeutralMode = IdleMode.kBrake; // What the lower shooter motor should do when not applied with any power (should always be brake while running to prevent overshooting target)
        public static final IdleMode upperShooterMotorNeutralMode = IdleMode.kBrake; // What the upper shooter motor should do when not applied with any power (should always be brake while running to prevent overshooting target)

        public static final IdleMode lowerShooterMotorIdleMode = IdleMode.kCoast; // What the lower shooter motor should so when the robot hasn't been initialized
        public static final IdleMode upperShooterMotorIdleMode = IdleMode.kCoast; // What the upper shooter motor should so when the robot hasn't been initialized

        /* Spped Profiling */
        public static final double ampScorerSpeed = 0.45;
        public static final double shooterScorerSpeed = 1;

        /* Shooter Voltage Compensation */
        public static final double voltageCompensation = 12.0; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)
    }

    public static final class ClimberConstants {
        /* Hardware ID from CAN */
        public static final int LeftClimbMotorID = 17; // Motor ID of the motor thats attched on the left climber of the robot
        public static final int RightClimbMotorID = 18; // Motor ID of the motor thats attached on the right climber of the robot

        /* Climber Current Limiting */
        public static final int rightClimbContinuousCurrentLimit = 60; // Climbers don't need to be running at maximum amperes due to the extension limit / extension length. Runs at 75%.
        public static final int leftClimbContinuousCurrentLimit = 60; // Climbers don't need to be running at maximum amperes due to the extension limit / extension length. Runs at 75%.

        /* Motor Inversions */
        public static final boolean leftClimbInvert = false;
        public static final boolean rightClimbInvert = false;

        /* Neutral Modes */ 
        public static final IdleMode leftClimbNeutralMode = IdleMode.kBrake;
        public static final IdleMode rightClimbNeutralMode = IdleMode.kBrake;

        /* PID Values for the Motors. Used to correct the error when trying to move the motors to a desired location */
        public static final double climbKP = 0.15; // Propotional: If there is error, move the motor propotional to the error
        public static final double climbKI = 0.0; // Intergral: If the error is taking too long to correct, move the motor faster
        public static final double climbKD = 0.0; // Derivative: If the motor is getting close to reaching the target, slow it down
        public static final double climbKFF = 0.0; // Force: Additional gain for creating offsets

        /* Speed Profiling */
        public static final double climbSpeed = 0.75; // Speed to default the climbers at (we have a reduced gearbox)

        /* Climber Voltage Compensation */
        public static final double voltageCompensation = 12.0; // For PID tuning, the max voltage that the PID will compensate for this value (for example at 12V your PID will tune for receiving for 12V, or the max battery output)
    }

    public static final class TankConstants {
        /* Hardware ID from CAN */
        public static final int FrontLeftID = 1; // Motor ID of the motor thats closest to the front in the left gearbox
        public static final int FrontRightID = 2; // Motor ID of the motor thats closest to the front in the right gearbox
        public static final int BackLeftID = 3; // Motor ID of the motor thats closest to the back in the left gearbox
        public static final int BackRightID = 4; // Motor ID of the motor thats closest to the back in the right gearbox

        /* Motor count on gearbox */
        public static final int LeftMotorCount = 2;
        public static final int RightMotorCount = 2;

        /* Speed Profiling */
        public static double ForwardReductionSpeed = 0; // Tuning for the forward speed of our drivetrain, if needed
        public static double RotationReductionSpeed = 0; // Tuning for the rotational speed of our drivetrain, if needed
    }

    public static final class ControllerRawButtons {
        public static final class XboxController {
            public enum Axis {
                kLeftX(0),
                kLeftY(1),
                kLeftTrigger(2),
                kRightTrigger(3),
                kRightX(4),
                kRightY(5);
            
                public final int value;
            
                Axis(int value) {
                    this.value = value;
                }
            
                public String toString() {
                    var name = this.name().substring(1); // Remove leading `k`
                    if (name.endsWith("Trigger")) {
                        return name + "Axis";
                    }

                    return name;
                }
            }

            /* Button Constants for Xbox Controllers */
            public enum Button {
                kA(1),
                kB(2),
                kX(3),
                kY(4),
                kLeftBumper(5),
                kRightBumper(6),
                kBack(7),
                kStart(8),
                kLeftStick(9),
                kRightStick(10);

                public final int value;
            
                Button(int value) {
                    this.value = value;
                }

                public String toString() {
                    var name = this.name().substring(1); // Remove leading `k`
                    if (name.endsWith("Bumper")) {
                        return name;
                    }

                    return name + "Button";
                }
            }
        }

        public final static class PS5Controller {
            public enum Button {
                kSquare(1),
                kCross(2),
                kCircle(3),
                kTriangle(4),
                kL1(5),
                kR1(6),
                kL2(7),
                kR2(8),
                kCreate(9),
                kOptions(10),
                kL3(11),
                kR3(12),
                kPS(13),
                kTouchpad(14);
            
                public final int value;
            
                Button(int index) {
                    this.value = index;
                }
            
                public String toString() {
                    var name = this.name().substring(1); // Remove leading `k`
                    if (this == kTouchpad) {
                        return name;
                    }
                    
                    return name + "Button";
                }
            }

            public enum Axis {
                kLeftX(0),
                kLeftY(1),
                kRightX(2),
                kRightY(5),
                kL2(3),
                kR2(4);
            
                public final int value;
            
                Axis(int index) {
                    value = index;
                }
            
                public String toString() {
                    var name = this.name().substring(1); // Remove leading `k`
                    if (name.endsWith("2")) {
                        return name + "Axis";
                    }

                    return name;
                }
            }
        }

        /* DPAD Angular Constants for Xbox Controllers */
        public static final int DPAD_NORTH = 0;
        public static final int DPAD_NORTHEAST = 45;
        public static final int DPAD_EAST = 90;
        public static final int DPAD_SOUTHEAST = 135;
        public static final int DPAD_SOUTH = 180;
        public static final int DPAD_SOUTHWEST = 225;
        public static final int DPAD_WEST = 270;
        public static final int DPAD_NORTHWEST = 315;
        public static final int DPAD_NOT_PRESSED = -1;
    }

    public static final class DriverConstants {

        /* Default ports for our two Controllers */
        public static final int driverControllerPort = 0; // Driver Controller only controlls the driving and maneuverability of the robot
        public static final int operatorControllerPort = 1; // Operator Controllers helps with all of the other mechanisms and subsystems attached on the robot.

        public static final double kDeadband = 0.1; // Default deband to help with stick drift on the controllers, recorded values we get is usually (+- 0.05)
        public static final boolean disableHAL = false; // Disables the HAL for the robot, used for testing purposes

        public static final boolean IS_ALLIANCE_RED = true;
        public static final boolean IS_ALLIANCE_BLUE = !IS_ALLIANCE_RED;
    }
}
