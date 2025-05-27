package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.PhotonPoseEstimator;
import frc.robot.utilities.PhotonPoseEstimator.ConstrainedSolvepnpParams;

public class Camera {
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final SimCameraProperties simProp;
    private final PhotonPoseEstimator poseEstimator;
    private final Optional<ConstrainedSolvepnpParams> constrainedPnpParams;
    private final Debouncer disabledDebouncer = new Debouncer(5.0, DebounceType.kFalling);

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Camera(String cameraName, Transform3d cameraToRobot) {
        simProp = new SimCameraProperties();
        setupCameraSim();

        camera = new PhotonCamera(cameraName);
        cameraSim = new PhotonCameraSim(camera, simProp);
        poseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, cameraToRobot);
        constrainedPnpParams = Optional.of(new ConstrainedSolvepnpParams(true, 0.0));
    }

    public void setupCameraSim() {
        simProp.setCalibration(640, 720, Rotation2d.fromDegrees(70));
        simProp.setFPS(28);
        simProp.setAvgLatencyMs(50);
    }

    /**
     * Clears all heading data in the buffer.
     */
    public void clearHeadingData() {
        poseEstimator.clearHeadingData();
    }

    /**
     * Adds reference poses to be utilized by the Photon pose estimator.
     * @param poseHistory Robot pose estimates from the last robot cycle.
     */
    public void addReferencePoses(List<PhotonPoseEstimator.TimestampedPose> poseHistory) {
        for (var pose : poseHistory) {
            poseEstimator.addHeadingData(pose.timestamp(), pose.pose().getRotation());
        }
    }

    /**
     * Refreshes the provided lists with new unread results from the camera. Note
     * that this method does not remove any elements from the supplied lists.
     * @param measurements A list of vision measurements to add to.
     * @param targets A list of targets to add to.
     */

    public void refresh(List<PhotonPoseEstimator.VisionMeasurement> measurements, List<Pose3d> targets) {
        // If we are disabled, use Constrained SolvePNP to estimate the robot's heading.
        boolean usingTrig = disabledDebouncer.calculate(false);
        poseEstimator.setPrimaryStrategy(usingTrig ? PoseStrategy.PNP_DISTANCE_TRIG_SOLVE : PoseStrategy.CONSTRAINED_SOLVEPNP);

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            // Get an estimate from the PhotonPoseEstimator.
            var estimate = poseEstimator.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);
            if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

            // Get the target AprilTag, and reject the measurement if the
            // tag is not configured to be utilized by the pose estimator.
            var target = estimate.get().targetsUsed.get(0);
            int id = target.fiducialId;
            //if (!useTag(id)) continue;

            // Get the location of the tag on the field.
            var tagLocation = kTagLayout.getTagPose(id);
            if (tagLocation.isEmpty()) continue;

            // Determine the distance from the camera to the tag.
            double distance = target.bestCameraToTarget.getTranslation().getNorm();

            // Calculate the pose estimation weights for X/Y location. As
            // distance increases, the tag is trusted exponentially less.
            double xyStd = (usingTrig ? 0.1 : 0.4) * distance * distance;

            // Calculate the angular pose estimation weight. If we're solving via trig, reject the heading estimate to ensure the pose estimator doesn't "poison" itself with
            // essentially duplicate data. Otherwise, weight the estimate similar to X/Y.
            double angStd = (usingTrig ? 0.1 : 0.14) * distance * distance;

            // Push the measurement to the supplied measurements list.
            measurements.add(
                new PhotonPoseEstimator.VisionMeasurement(
                    estimate.get().estimatedPose.toPose2d(),
                    estimate.get().timestampSeconds,
                    VecBuilder.fill(xyStd, xyStd, angStd)
                )
            );

            // Push the location of the tag to the targets list for telemetry.
            targets.add(tagLocation.get());
        }
    }

    /**
     * Returns {@code true} if an AprilTag should be utilized.
     * @param id The ID of the AprilTag.
     */
    private boolean useTag(int id) {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? (id >= 6 && id <= 11) : (id >= 17 && id <= 22);
    }
}