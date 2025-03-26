package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.RectanglePoseArea;
import frc.robot.utilities.LimelightHelper.RawFiducial;
import frc.robot.utilities.LimelightHelper;

public class Limelight extends SubsystemBase {

  private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight-left");
  private final NetworkTableEntry targetInViewEntry = limeTable.getEntry("TargetInView");
  private final NetworkTableEntry tplEntry = limeTable.getEntry("pipeline");
  public boolean targetInView;
  private int count = 0;
  double yawAngleDegrees;
  double xDistanceMeters;
  double yDistanceMeters;
  double zDistanceMeters;
  Pose3d pose3D;
  protected int timeSinceValid = 0;
  private LinearFilter filteredYawDegrees = LinearFilter.movingAverage(10);

  public Limelight() {
    System.out.println("-------- Start Limelight\n");
  }

  // This method is encapsulated so it can be overriden for simulation
  protected double[] getTargetPoseCameraSpace() {
    return LimelightHelper.getTargetPose_CameraSpace("limelight-left");
  }

  protected Transform2d getTransposeCameraToRobotSpace() {
    return new Transform2d(-1 * 0.08, 0, new Rotation2d());
  }

  // Check if limelight is out of field of view
  public boolean isAllZeros(double[] arr) {
    for (int i = 0; i < arr.length; i++) {
      if (arr[i] != 0) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void periodic() {
    targetInView = targetInViewEntry.getDouble(0) >= 1.0;
    SmartDashboard.putString("plType", LimelightHelper.getCurrentPipelineType("limelight-left"));
    double[] cameraTargetPose = getTargetPoseCameraSpace();
    if (cameraTargetPose.length > 0) {
      if (isAllZeros(cameraTargetPose)) {
        timeSinceValid++;
      } else {
        xDistanceMeters = cameraTargetPose[0];
        yDistanceMeters = cameraTargetPose[1];
        zDistanceMeters = cameraTargetPose[2];
        yawAngleDegrees = cameraTargetPose[4];
        filteredYawDegrees.calculate(yawAngleDegrees);
        timeSinceValid = 0;
      }
    } else {
      timeSinceValid++;
    }

    if (count % 15 == 0) {
      SmartDashboard.putNumber("xDis", xDistanceMeters);
      SmartDashboard.putNumber("yDis", yDistanceMeters);
      SmartDashboard.putNumber("zDis", zDistanceMeters);
      SmartDashboard.putNumber("yaw", yawAngleDegrees);
      SmartDashboard.putNumber("TV", targetInView ? 1 : 0);
      SmartDashboard.putNumber("LLPl", getLimelightPipeline());
    }
    count++;
    if (count == 150000) {
      count = 0;
    }

  }

  public void setLimelightPipeline(int pipeline) {
    System.out.println("Setting Limelight pipeline to " + pipeline);
    limeTable.getEntry("pipeline").setNumber(pipeline);
  }

  public int getLimelightPipeline() {
    return (int) tplEntry.getDouble(-1);
  }

  public double getYawAngleDegrees() {
    return yawAngleDegrees;
  }

  public double getxDistanceMeters() {
    return xDistanceMeters;
  }

  public double getyDistanceMeters() {
    return yDistanceMeters;
  }

  public double getzDistanceMeters() {
    return zDistanceMeters;
  }

  public double getFilteredYawDegrees() {
    return filteredYawDegrees.lastValue();
  }

  public int getTimeSinceValid() {
    return timeSinceValid;
  }
}
