package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.LimelightHelper.PoseEstimate;
import frc.robot.utilities.constants.FieldConstants;

public class LimelightSubsystem extends SubsystemBase {
  
   public enum Limelight{
        // Planned on having 3 cameras, only used 1
        FUNNEL("limelight-funnel", new Pose3d()), 
        ELEVATOR("limelight-elevator", new Pose3d(0.286, -0.162, 0.233, new Rotation3d(0, Math.toRadians(27), Math.toRadians(180.0)))),
        LEFT("limelight-left", new Pose3d());

        String name;
        Pose3d mountPose;

        private Limelight(String name, Pose3d mountPose){
            this.name = name;
            this.mountPose = mountPose;
        }
    }

    public enum FunnelPipeline{
        DETECTOR_LARGEST(0), DETECTOR_LEFTMOST(1), DETECTOR_RIGHTMOST(2);

        int index;

        private FunnelPipeline(int index){
            this.index = index;
        }
    }

    public enum ElevatorPipeline{
        FIDUCIAL(0);

        int index;

        private ElevatorPipeline(int index){
            this.index = index;
        }
    }

    public enum LeftPipeline{
        FIDUCIAL(0);

        int index;

        private LeftPipeline(int index){
            this.index = index;
        }
    }

    public enum LEDMode{
        OFF, ON, BLINK;
    }

    public static double getTX(Limelight limelight){
        return LimelightHelper.getTX(limelight.name);
    }

    public static double getTY(Limelight limelight){
        return LimelightHelper.getTY(limelight.name);
    }

    public static double getTA(Limelight limelight){
        return LimelightHelper.getTA(limelight.name);
    }

    public static boolean getTV(Limelight limelight){
        return LimelightHelper.getTV(limelight.name);
    }

    public static double getLatency_Pipeline(Limelight limelight){
        return LimelightHelper.getLatency_Pipeline(limelight.name);
    }

    public static double getLatency_Capture(Limelight limelight){
        return LimelightHelper.getLatency_Capture(limelight.name);
    }

    public static Pose2d getBotPose2d(Limelight limelight){
        return LimelightHelper.getBotPose2d_wpiBlue(limelight.name);
    }

    public static Pose3d getBotPose3d(Limelight limelight){
        return LimelightHelper.getBotPose3d_wpiBlue(limelight.name);
    }

    public static Pose2d getBotPose2d_TargetSpace(Limelight limelight){
        return LimelightHelper.getBotPose3d_TargetSpace(limelight.name).toPose2d();
    }

    public static Pose3d getBotPose3d_TargetSpace(Limelight limelight){
        return LimelightHelper.getBotPose3d_TargetSpace(limelight.name);
    }

    public static double getDistanceToTargetMeters(double targetHeight, Limelight limelight){
        double goal_theta = limelight.mountPose.getRotation().getY() + Math.toRadians(getTY(Limelight.ELEVATOR));
        double height_diff = targetHeight - limelight.mountPose.getZ();

        return height_diff / Math.tan(goal_theta);
    }

    public static void updateEstimateWithValidMeasurements(Limelight limelight, SwerveDrivePoseEstimator poseEstimator){
        validateVisionMeasurement(limelight, LimelightHelper.getBotPoseEstimate_wpiBlue(limelight.name), poseEstimator);
    }

    private static void validateVisionMeasurement(Limelight limelight, PoseEstimate estimate, SwerveDrivePoseEstimator poseEstimator){
        if (estimate.pose.getX() == 0.0 || !FieldConstants.fieldArea.isPoseWithinArea(estimate.pose)) {
          return;
        }

        double latency = getLatency_Pipeline(limelight) + getLatency_Capture(limelight);
        latency = latency / 1000.0;

        double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
            .getDistance(estimate.pose.getTranslation());

        if (estimate.tagCount != 0) {
            double xyStds;
            if (estimate.tagCount >= 2) {
                xyStds = 0.3;
            } else if (estimate.avgTagArea > 0.8 && poseDifference < 0.3) {
                xyStds = 0.7;
            } else if (estimate.avgTagArea < 0.1 && poseDifference < 0.9) {
                xyStds = 0.7;
            } else {
                return;
            }
            poseEstimator.addVisionMeasurement(estimate.pose, Timer.getFPGATimestamp() - latency, VecBuilder.fill(xyStds, xyStds, 99999999));
        }
    }

    public static int getCurrentPipeline(Limelight limelight){
        return (int) LimelightHelper.getCurrentPipelineIndex(limelight.name);
    }

    public static void setLEDMode(Limelight limelight, LEDMode ledMode){
        switch (ledMode) {
            case OFF:
            LimelightHelper.setLEDMode_ForceOff(limelight.name);
            break;
            case ON:
            LimelightHelper.setLEDMode_ForceOn(limelight.name);
            break;
            case BLINK:
            LimelightHelper.setLEDMode_ForceBlink(limelight.name);
            break;
        }
    }

    public static void setPriorityTagID(int id, Limelight limelight){
      LimelightHelper.setPriorityTagID(limelight.name, id);
    }

    public static void setPipeline(Limelight limelight, int index){
      LimelightHelper.setPipelineIndex(limelight.name, index);
    }

    public static void setPipeline(FunnelPipeline pipeline){
        setPipeline(Limelight.FUNNEL, pipeline.index);
    }

    public static void setPipeline(ElevatorPipeline pipeline){
        setPipeline(Limelight.ELEVATOR, pipeline.index);
    }

    public static void setPipeline(LeftPipeline pipeline){
        setPipeline(Limelight.LEFT, pipeline.index);
    }

    public Command intakePipelineCommand(FunnelPipeline pipeline){
        return Commands.runOnce(() -> setPipeline(pipeline));
    }
    
    public Command frontPipelineCommand(ElevatorPipeline pipeline){
        return Commands.runOnce(() -> setPipeline(pipeline));
    }

    public Command backPipelineCommand(LeftPipeline pipeline){
        return Commands.runOnce(() -> setPipeline(pipeline));
    }
}