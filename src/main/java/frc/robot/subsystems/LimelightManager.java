package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.constants.Constants;

public class LimelightManager extends SubsystemBase{

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2
    }

    private MegaTagMode megaTagMode;
    private int maxTagCount;

    public LimelightManager() {
        for (Limelight.Camera camera : Limelight.LimelightCameras) {
            Pose3d robotRelativePose = camera.getLocation();
            LimelightHelper.setCameraPose_RobotSpace(
                camera.getName(), 
                robotRelativePose.getX(), 
                -robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getX()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getY()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getZ())
            );
        }

        maxTagCount = 0;

        megaTagMode = MegaTagMode.MEGATAG1;
        CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(Constants.LimelightConstants.MT1_STDEVS);

        setIMUMode(1);
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
    }

    public void setTagWhitelist(int... ids) {
        for (Limelight.Camera camera : Limelight.LimelightCameras) {
            LimelightHelper.SetFiducialIDFiltersOverride(camera.getName(), ids);
        }
    }

    public void setIMUMode(int mode) {
        for (Limelight.Camera camera : Limelight.LimelightCameras) {
            LimelightHelper.SetIMUMode(camera.getName(), mode);
        }
    }

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    public MegaTagMode getMTmode() {
        return megaTagMode;
    }

    public LimelightHelper.PoseEstimate getMegaTag1PoseEstimate(String limelightName) {
        return Robot.isBlue() 
            ? LimelightHelper.getBotPoseEstimate_wpiBlue(limelightName)
            : LimelightHelper.getBotPoseEstimate_wpiRed(limelightName);
    }

    private LimelightHelper.PoseEstimate getMegaTag2PoseEstimate(String limelightName) {
        LimelightHelper.SetRobotOrientation(
            limelightName, 
            (CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
            0, 
            0, 
            0, 
            0, 
            0
        );
        return Robot.isBlue() 
            ? LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    }

    @Override
    public void periodic() {
        this.maxTagCount = 0;

        for (Limelight.Camera camera : Limelight.LimelightCameras) {
            if (camera.isEnabled()) {
                LimelightHelper.PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                    ? getMegaTag2PoseEstimate(camera.getName())
                    : getMegaTag1PoseEstimate(camera.getName());

                Vector<N3> stdevs = (megaTagMode == MegaTagMode.MEGATAG2)
                    ? Constants.LimelightConstants.MT2_STDEVS
                    : Constants.LimelightConstants.MT1_STDEVS;
                
                if (poseEstimate != null && poseEstimate.tagCount > 0) {
                    CommandSwerveDrivetrain.getInstance().addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdevs);
                    SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", true);
                    SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", poseEstimate.tagCount);
                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                }
                else {
                    SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", false);
                    SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", 0);
                }
            }
        }

        SmartDashboard.putString("Vision/Megatag Mode", this.megaTagMode.toString());
    }
}