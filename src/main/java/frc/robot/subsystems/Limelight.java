package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.constants.Constants;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  NetworkTable table;

  NetworkTableEntry tx, ty, ta, tv;
  NetworkTableEntry ledMode, camMode, pipeline;

  ShuffleboardTab tab;
  GenericEntry distanceToTargetX;
  GenericEntry distanceToTargetY;
  GenericEntry targetVisible;

  String name;
  boolean backward;
  boolean isTestSubsystem;
  double limelightHeight, limelightAngle;
  boolean enabled;
  int currentPipeline = -1;
  boolean isNetworkTableConnected;
  Map<Double, LimelightHelper.LimelightTarget_Fiducial> llFiducialMap = new HashMap<Double, LimelightHelper.LimelightTarget_Fiducial>();
  Pose2d limelightPose = new Pose2d();

  //The distance from where you want to calculate from, should always be calculated with WPI coordinates (front is positive X)
  Translation2d offset;

  private Limelight(String limelightName, double limelightHeightMeters, double limelightAngleDegrees, double xOffsetMeters, double yOffsetMeters, boolean facingBackward, boolean isTestSubsystem, boolean enabled) {
    name = limelightName;
    limelightHeight = limelightHeightMeters;
    limelightAngle = limelightAngleDegrees;
    offset = new Translation2d(xOffsetMeters, yOffsetMeters);
    backward = facingBackward;
    this.isTestSubsystem = isTestSubsystem;
    this.enabled = enabled;

    if (enabled) {
      table = NetworkTableInstance.getDefault().getTable(name);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      
      ledMode = table.getEntry("ledMode");
      camMode = table.getEntry("camMode");
      pipeline = table.getEntry("pipeline");

      if (Constants.debug) {
        tab = Shuffleboard.getTab(name);
        targetVisible = tab.add("Target Visible", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).getEntry();
        distanceToTargetX = tab.add("Target X", 0).withPosition(0, 1).getEntry();
        distanceToTargetY = tab.add("Target Y", 0).withPosition(0, 2).getEntry();
      }
    }
  }

  public enum CamMode {
    VisionProcessor,
    DriverCamera;
  }

  public void updateBotposeWpiBlue() {
    if (enabled && isNetworkTableConnected) {
      if (getTargetVisible()) {
        limelightPose = LimelightHelper.getBotPose2d_wpiBlue(name);
      }
    }
  }

  public Pose2d getBotposeWpiBlue() {
    return limelightPose;
  }

  public int getNumTargets() {
    if (enabled && isNetworkTableConnected) {
      final int numTargets = llFiducialMap.size();
      Logger.recordOutput(name + "/NumberTargets", numTargets);
      return numTargets;
    }

    return 0;
  }

  public double getTotalLatency() {
    if (enabled && isNetworkTableConnected) {
      return LimelightHelper.getLatency_Capture(name) + LimelightHelper.getLatency_Pipeline(name);
    }

    return 0;
  }

  public void refreshOdometry() {
    String json = LimelightHelper.getJSONDump(name);
    llFiducialMap.clear();
    int nextPos = 0;
    while ((nextPos = json.indexOf("\"fID\":", nextPos)) != -1) {
      int startIndex = nextPos + 6;
      nextPos++; // don't get stuck in a loop if there is no tx

      if ((nextPos = json.indexOf(',', nextPos)) != -1) {
        LimelightHelper.LimelightTarget_Fiducial fiducial = new LimelightHelper.LimelightTarget_Fiducial();
        fiducial.fiducialID = Double.valueOf(json.substring(startIndex, nextPos));
        if ((nextPos = json.indexOf("\"tx\":", nextPos)) != -1) {
          startIndex = nextPos + 5;
          if ((nextPos = json.indexOf(',', nextPos)) != -1) {
            fiducial.tx = Double.parseDouble(json.substring(startIndex, nextPos));
            if ((nextPos = json.indexOf("\"ty\":", nextPos)) != -1) {
              startIndex = nextPos + 5;
              if ((nextPos = json.indexOf(',', nextPos)) != -1) {
                fiducial.ty = Double.parseDouble(json.substring(startIndex, nextPos));
                llFiducialMap.put(fiducial.fiducialID, fiducial);
              }
            }
          }
        }
      }
    }
  }

  public LimelightHelper.LimelightTarget_Fiducial getTag(double fID) {
    return llFiducialMap.get(fID);
  }

  public boolean getSpecifiedTagVisible(int fID1) {
    return getTag(fID1) != null;
  }

  public double getHorizontalDegToTarget() {
    if (enabled && isNetworkTableConnected) {
      return tx.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getVerticalDegToTarget() {
    if (enabled && isNetworkTableConnected) {
      return ty.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getTargetArea() {
    if (enabled && isNetworkTableConnected) {
      return ta.getDouble(0);
    } else {
      return 0;
    }
  }

  public boolean getTargetVisible() {
    if (enabled && isNetworkTableConnected) {
      return tv.getDouble(0.0) == 1.0;
    } else {
      return false;
    }
  }

  public void setCamMode(CamMode mode) {
    if (enabled && isNetworkTableConnected) {
      if (mode == CamMode.VisionProcessor) {
        camMode.setNumber(0);
      } else if (mode == CamMode.DriverCamera) {
        camMode.setNumber(1);
      }
    }
  }

  // All distances use WPILib coordinates (where x is perpendicular to the target and y is parallel to the target)
  public double getTargetHeight(int pipelineIdx, double yDeg) {
    if (Constants.LimelightConstants.tagPipelinesHeights.containsKey(pipelineIdx)) {
      return Constants.LimelightConstants.tagPipelinesHeights.get(pipelineIdx);
    }

    return -1; // invalid pipeline
  }

  // Angles in degrees distance y formula referenced from: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public Translation2d calcTargetPos(double targetHeight, double yDeg, double xDeg) {
    double distanceX = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(yDeg + limelightAngle));
    double distanceY = distanceX * Math.tan(Math.toRadians(xDeg));
    Translation2d toReturn = new Translation2d(distanceX, distanceY).plus(offset);

    if (backward) {
      toReturn = toReturn.rotateBy(Rotation2d.fromDegrees(180));
    }

    return toReturn;
  }

  public Translation2d getTargetPosRobotRelative() {
    if (enabled && isNetworkTableConnected) {
      if (getTargetVisible()) {
        double yDeg = getVerticalDegToTarget();
        double xDeg = getHorizontalDegToTarget();

        int pipelineIdx = (int) pipeline.getInteger(0);
        double targetHeight = getTargetHeight(pipelineIdx, yDeg);

        if (targetHeight != -1) {
          return calcTargetPos(targetHeight, yDeg, xDeg);
        }

        return new Translation2d(0, 0);
      }

      return new Translation2d(0, 0);
    }
    
    return new Translation2d(0, 0);
  }

  public double getHorizontalDistToTarget() {
    double distanceX = (getTargetHeight(currentPipeline, getHorizontalDegToTarget()) - limelightHeight) / Math.tan(Math.toRadians(getHorizontalDegToTarget() + limelightAngle));
    double distanceY = distanceX * Math.tan(Math.toRadians(getVerticalDegToTarget()));

    return distanceY;
  }

  public void activateRetroReflective() {
    switchPipeline(0);
  }

  public void activateAprilTag() {
    switchPipeline(1);
  }

  private void switchPipeline(int pipelineIdx) {
    if (enabled && (currentPipeline != pipelineIdx) && isNetworkTableConnected) {
      pipeline.setNumber(pipelineIdx);
      currentPipeline = pipelineIdx;
    }
  }
}