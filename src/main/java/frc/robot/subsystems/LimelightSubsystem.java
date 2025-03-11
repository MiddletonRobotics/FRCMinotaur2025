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
    private static Limelight leftElevator = new Limelight("left-elevator");
    private static Limelight rightElevator = new Limelight("right-elevator");

    public LimelightSubsystem() {}

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
}