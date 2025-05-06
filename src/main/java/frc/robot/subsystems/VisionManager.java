package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.utilities.PhotonPoseEstimator.TimestampedPose;
import frc.robot.utilities.PhotonPoseEstimator.VisionMeasurement;

public class VisionManager {
    private Camera[] cameras;
    public final AprilTagFieldLayout kTagLayout;
    

    private final List<Pose2d> estimates = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();

    public VisionManager() {
        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        cameras = new Camera[] {
            new Camera("elevatorLeft", new Transform3d(new Translation3d(0.178, 0.285 + 0.0214, 0.145), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-30.76)))),    
            new Camera("elevatorRight", new Transform3d(new Translation3d(0.178 - 0.0243, -0.285, 0.145), new Rotation3d(0, Math.toRadians(-30), Math.toRadians(30.76))))
        };

        // Hit the undocumented Photon Turbo Button™: https://github.com/PhotonVision/photonvision/pull/1662
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    /**
     * Resets cached measurements utilized by the pose estimators for seeding. It is
     * recommended to call this method after resetting the robot's pose or rotation.
     */

    public void reset() {
        for (var camera : cameras) camera.clearHeadingData();
    }

    /**
     * Gets unread results from all cameras.
     * @param poseHistory Robot pose estimates from the last robot cycle.
     */

    public VisionMeasurement[] getUnreadResults(List<TimestampedPose> poseHistory) {
        List<VisionMeasurement> measurements = new ArrayList<>();

        estimates.clear();
        targets.clear();

        for (var camera : cameras) {
            camera.addReferencePoses(poseHistory);
            camera.refresh(measurements, targets);
        }

        estimates.addAll(measurements.stream().map(m -> m.pose()).toList());
        return measurements.stream().toArray(VisionMeasurement[]::new);
    }
}