package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelper;

public class Limelight extends SubsystemBase {
    public enum LED_MODE {
        PIPELINE, OFF, BLINK, ON
    }

    public enum STREAM_MODE {
        STANDARD, PIP_MAIN, PIP_SECOND
    }

    private boolean ignoreFlLimelight = false;
    private boolean ignoreFrLimelight = false;
    private boolean ignoreAllLimelights = false;

    private StructPublisher<Pose2d> frNT = NetworkTableInstance.getDefault().getStructTopic("flNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> flNT = NetworkTableInstance.getDefault().getStructTopic("frNT", Pose2d.struct).publish();

    private LimelightHelper.PoseEstimate limeFL, limeFR;
    private LimelightHelper.PoseEstimate limeFLPrev, limeFRPrev;

    private Notifier notifier;

    private CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.system();
    private static Limelight instance = null;

    /**
     * Constructs a {@link Limelight} subsytem instance
     */
    private Limelight() {
        this.startThread();
    }

    /**
     * Sets the stream mode of the limelight
     *
     * @param limelightName The name of the limelight
     * @param mode {@link STREAM_MODE} of the limelight
     */
    public void setStreamMode(String limelightName, STREAM_MODE mode) {
        if (mode == STREAM_MODE.STANDARD) {
            LimelightHelper.setStreamMode_Standard(limelightName);
        } else if (mode == STREAM_MODE.PIP_MAIN) {
            LimelightHelper.setStreamMode_PiPMain(limelightName);
        } else if (mode == STREAM_MODE.PIP_SECOND) {
            LimelightHelper.setStreamMode_PiPSecondary(limelightName);
        }
    }

    /**
     * Sets the pipeline of the limelight
     *
     * @param limelightName The name of the limelight
     * @param pipeline The pipeline index
     */
    public void setPipeline(String limelightName, int pipeline) {
        LimelightHelper.setPipelineIndex(limelightName, pipeline);
    }

    /**
     * Sets the LED mode of the limelight
     *
     * @param limelightName The name of the limelight
     * @param mode The LED mode
     */
    public void setLedMode(String limelightName, LED_MODE mode) {
        if (mode == LED_MODE.PIPELINE) {
            LimelightHelper.setLEDMode_PipelineControl(limelightName);
        } else if (mode == LED_MODE.OFF) {
            LimelightHelper.setLEDMode_ForceOff(limelightName);
        } else if (mode == LED_MODE.BLINK) {
            LimelightHelper.setLEDMode_ForceBlink(limelightName);
        } else if (mode == LED_MODE.ON) {
            LimelightHelper.setLEDMode_ForceOn(limelightName);
        }
    }

    /**
     * Starts the Limelight odometry thread
     */
    private void startThread() {
        notifier = new Notifier(this::loop);
        notifier.startPeriodic(0.02);
    }

    /**
     * The main loop of the Limelight odometry thread
     */
    private void loop() {
        if (swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > 720) {
            ignoreAllLimelights = true;
        } else {
            ignoreAllLimelights = false;
        }

        /*

        updateFrontLeft();
        updateFrontRight();

        */
    }

    /**
     * Updates the odometry for the front left limelight
     */
    private void updateFrontLeft() {
        LimelightHelper.SetRobotOrientation(
            "limelight-left",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeFL = LimelightHelper.getBotPoseEstimate_wpiBlue("limelight-left");

        if (limeFL != null && limeFL.pose != null) {
            ignoreFlLimelight = !poseInField(limeFL.pose) ||
                (Math.abs(LimelightHelper.getBotPose3d_wpiBlue("limelight-left").getZ()) > 0.4) ||
                (LimelightHelper.getTA("limelight-left") < 0.1) ||
                (limeFLPrev != null && (limeFL.pose.getTranslation().getDistance(limeFLPrev.pose.getTranslation()) /
                    (limeFL.timestampSeconds - limeFLPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFLPrev != null && (limeFL.pose.getTranslation()
                    .getDistance(limeFLPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeFL.rawFiducials.length > 0 && limeFL.rawFiducials[0].ambiguity > 0.5 &&
                    limeFL.rawFiducials[0].distToCamera > 4.0) ||
                limeFL.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimelights && !ignoreFlLimelight) {
                SmartDashboard.putBoolean("FLpose", true);
                flNT.set(limeFL.pose);

                swerve.addVisionMeasurement(
                    limeFL.pose,
                    Utils.fpgaToCurrentTime(limeFL.timestampSeconds)
                );
            } else {
                SmartDashboard.putBoolean("FLpose", false);
            }

            limeFLPrev = limeFL;
        }
    }

    /**
     * Updates the odometry for the front right limelight
     */
    private void updateFrontRight() {
        LimelightHelper.SetRobotOrientation(
            "limelight-right",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );

        limeFR = LimelightHelper.getBotPoseEstimate_wpiBlue("limelight-right");

        if (limeFR != null && limeFR.pose != null) {
            ignoreFrLimelight = !poseInField(limeFR.pose) ||
                (Math.abs(LimelightHelper.getBotPose3d_wpiBlue("limelight-right").getZ()) > 0.4) ||
                (LimelightHelper.getTA("limelight-right") < 0.1) ||
                (limeFRPrev != null && (limeFR.pose.getTranslation().getDistance(limeFRPrev.pose.getTranslation()) /
                    (limeFR.timestampSeconds - limeFRPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFRPrev != null && (limeFR.pose.getTranslation()
                    .getDistance(limeFRPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeFR.rawFiducials.length > 0 && limeFR.rawFiducials[0].ambiguity > 0.5 &&
                    limeFR.rawFiducials[0].distToCamera > 4.0) ||
                limeFR.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimelights && !ignoreFrLimelight) {
                SmartDashboard.putBoolean("FRpose", true);
                frNT.set(limeFR.pose);

                swerve.addVisionMeasurement(
                    limeFR.pose,
                    Utils.fpgaToCurrentTime(limeFR.timestampSeconds)
                );
            } else {
                SmartDashboard.putBoolean("FRpose", false);
            }

            limeFRPrev = limeFR;
        }
    }

    /**
     * Checks if a pose is inside the field dimensions
     *
     * @param pose The {@link Pose2d} to check
     * @return True if the pose is inside the field dimensions, false otherwise
     */
    private boolean poseInField(Pose2d pose) {
        return pose.getTranslation().getX() > 0 &&
            pose.getTranslation().getX() < 17.55 &&
            pose.getTranslation().getY() > 0 &&
            pose.getTranslation().getY() < 8.05;
    }

    /**
     * Gets the {@link Limelight} subsystem instance
     *
     * @return The {@link Limelight} subsystem instance
     */
    public static Limelight system() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }
}