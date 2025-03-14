package frc.robot.subsystems;

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
import frc.robot.utilities.constants.FieldConstants;

public class LimelightSubsystem extends SubsystemBase {
    private static Limelight leftElevator;
    private static Limelight rightElevator;
    public static String limelightUsed;

    public LimelightSubsystem() {
        leftElevator = new Limelight("left-elevator");
        rightElevator = new Limelight("right-elevator");

        //leftElevator.set
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

    public String chooseLimelight(){
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
}