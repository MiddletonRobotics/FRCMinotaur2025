package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.LimelightHelper.PoseEstimate;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.FieldConstants;

public class LimelightSubsystem extends SubsystemBase {
    private static Limelight leftElevator;
    private static Limelight rightElevator;
    public static String limelightUsed;

    public LimelightSubsystem() {
        leftElevator = new Limelight("left-elevator");
        rightElevator = new Limelight("right-elevator");

        leftElevator.setLimelightPosition(
            "left-elevator", 
            Constants.LimelightConstants.leftLimelightForward.in(Meters), 
            Constants.LimelightConstants.leftLimelightLateral.in(Meters), 
            Constants.LimelightConstants.leftLimelightHeight.in(Meters), 
            Constants.LimelightConstants.leftLimelightRoll.in(Degrees), 
            Constants.LimelightConstants.leftLimelightPitch.in(Degrees), 
            Constants.LimelightConstants.leftLimelightYaw.in(Degrees)
        );

        rightElevator.setLimelightPosition(
            "left-elevator", 
            Constants.LimelightConstants.rightLimelightForward.in(Meters), 
            Constants.LimelightConstants.rightLimelightLateral.in(Meters), 
            Constants.LimelightConstants.rightLimelightHeight.in(Meters), 
            Constants.LimelightConstants.rightLimelightRoll.in(Degrees), 
            Constants.LimelightConstants.rightLimelightPitch.in(Degrees), 
            Constants.LimelightConstants.rightLimelightYaw.in(Degrees)
        );
    }

    @Override
    public void periodic() {
        leftElevator.update();
        rightElevator.update();
    }

    public static Limelight getLeftElevator() {
        return leftElevator;
    }

    public static Limelight getRightElevator() {
        return rightElevator;
    }

    public String chooseLimelight() {
        double limelightLeftAvgTagArea = NetworkTableInstance.getDefault().getTable("left-elevator").getEntry("botpose").getDoubleArray(new double[11])[10];
        double limelightRightAvgTagArea = NetworkTableInstance.getDefault().getTable("right-elevator").getEntry("botpose").getDoubleArray(new double[11])[10];
        
        SmartDashboard.putNumber("Left Limelight Tag Area", limelightLeftAvgTagArea);
        SmartDashboard.putNumber("Right Limelight Tag Area", limelightRightAvgTagArea);    

        if(limelightLeftAvgTagArea > limelightRightAvgTagArea){
            limelightUsed = "left-elevator";
        } else{
            limelightUsed = "right-elevator";
        }
        
        SmartDashboard.putString("Limelight Used", limelightUsed);
        return limelightUsed;
    }

    public void setRobotPosition(String tableKey, double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        LimelightHelper.SetRobotOrientation(tableKey, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public void setRobotPosition(String tableKey, double yaw) {
        LimelightHelper.SetRobotOrientation(tableKey, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}