package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.constants.FieldConstants;

public class Limelight {
    private final NetworkTable table;
  
    private double targetDetected;

    private double targetXOffset;
    private double targetYOffset;

    private double[] currentBotPoseBlue = new double[6];
    private double[] lastBotPoseBlue = new double[6];
    private double[] currentRobotTargetSpacePose = new double[6];
    private double[] lastRobotTargetSpacePose = new double[6];
    private double[] currentTargetCameraSpacePose = new double[6];
    private double[] lastTargetCameraSpacePose = new double[6];

    private double currentAprilTagID;
    private double lastAprilTagID;

    private final NetworkTableEntry ledMode;
    private final NetworkTableEntry cameraMode;
    private final NetworkTableEntry pipeline;
    private final NetworkTableEntry priorityID;

    /**
     * Creates a new Limelight.
     * 
     * @param tableKey The key/name assigned to the desired Limglight on NetworkTables.
     */

    public Limelight(String tableKey) {
        table = NetworkTableInstance.getDefault().getTable(tableKey);

        ledMode = table.getEntry("ledMode");
        cameraMode = table.getEntry("cameraMode");
        pipeline = table.getEntry("pipeline");
        priorityID = table.getEntry("priorityid");
    }

    /**
     * Updates all values received from NetworkTables for this Limelight.
     */
    public void update() {
        targetDetected = table.getEntry("tv").getDouble(0);

        targetXOffset = table.getEntry("tx").getDouble(0);
        targetYOffset = table.getEntry("ty").getDouble(0);

        currentBotPoseBlue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        handleRawPose(currentBotPoseBlue, lastBotPoseBlue);
        currentRobotTargetSpacePose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        handleRawPose(currentRobotTargetSpacePose, lastRobotTargetSpacePose);

        currentTargetCameraSpacePose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        handleRawPose(currentTargetCameraSpacePose, lastTargetCameraSpacePose);

        currentAprilTagID = table.getEntry("tid").getDouble(0);

        if(currentAprilTagID != 0 && currentAprilTagID != -1) {
            lastAprilTagID = currentAprilTagID;
        }
  }

    /**
     * @return Whether the Limelight can see any valid targets
     */

    public boolean getTargetDetected() {
        return targetDetected == 1;
    }

    /**
     * @return The horizontal offset from the crosshair to the target
     */

    public double getXOffset() {
        return targetXOffset;
    }

    /**
     * @return The vertical offset from the crosshair to the target
     */

    public double getYOffset() {
        return targetYOffset;
    }

    /**
     * @return The last received non-empty robot pose with the origin at the right-hand side of the blue alliance driverstation
     *         if any. All-zero pose if none has been received yet.
     */

    public Pose2d getLastBotPoseBlue() {
        return new Pose2d(lastBotPoseBlue[0], lastBotPoseBlue[1], Rotation2d.fromDegrees(lastBotPoseBlue[5]));
    }

    /**
     * @return The current robot pose with the origin at the right-hand side of the blue alliance driverstation
     *         being received if any. Null if none is being received.
     */
    public Pose2d getCurrentBotPoseBlue() {
        SmartDashboard.putNumber("Limelight Pose X", currentBotPoseBlue[0]);
    
        if(currentBotPoseBlue != new double[6]) {
            return new Pose2d(currentBotPoseBlue[0], currentBotPoseBlue[1], Rotation2d.fromDegrees(currentBotPoseBlue[5]));
        }
        else {
            return null;
        }
    }

    /**
     * @return The last received non-empty robot pose with the target as the origin if any.
     *         All-zero pose if none has been received yet.
     */

    public Pose3d getRobotTargetSpacePose() {
        return new Pose3d(lastRobotTargetSpacePose[0], lastRobotTargetSpacePose[1], lastRobotTargetSpacePose[2], new Rotation3d(lastRobotTargetSpacePose[3], lastRobotTargetSpacePose[4], lastRobotTargetSpacePose[5]));
    }

    /**
     * @return The last received non-empty camera pose with the target as the origin if any.
     *         All-zero pose if none has been received yet.
     */
    
    public Pose3d getTargetCameraSpacePose() {
        return new Pose3d(lastTargetCameraSpacePose[0], lastTargetCameraSpacePose[1], lastTargetCameraSpacePose[2], new Rotation3d(lastTargetCameraSpacePose[3], lastTargetCameraSpacePose[4], lastTargetCameraSpacePose[5]));
    }

    /**
     * @return The ID of the last identified primary in-view AprilTag
     */

    public double getAprilTagID() {
        return lastAprilTagID;
    }

    /**
     * Sets the Limelight's LED state.
     * 
     * @param mode
     * 
     * <p>0: Use the LED mode set in the current pipeline
     * <p>1: Force off
     * <p>2: Force blink
     * <p>3: Force on
     */

    public void setLedMode(Number mode) {
        ledMode.setNumber(mode);
    }
  
    /**
     * Sets the Limelight's operation mode.
     * 
     * @param mode
     * 
     * <p>0: Vision processor
     * <p>1: Driver camera (Increases exposure, disables vision processing)
     */

    public void setCameraMode(Number mode) {
        cameraMode.setNumber(mode);
    }

    /**
     * Sets the Limelight's current pipeline.
     * 
     * @param pipelineNumber The desired pipeline number (0-9)
     */

    public void setPipeline(Number pipelineNumber) {
        pipeline.setNumber(pipelineNumber);
    }

    public void setPriorityID(Number id) {
        priorityID.setNumber(id);
    }

    public void handleRawPose(double[] rawPose, double[] processedPose) {
        if (rawPose != new double[6]) {
            for (int i = 0; i < 6; i++) {
                processedPose[i] = rawPose[i];
            }
        }
    }
}
