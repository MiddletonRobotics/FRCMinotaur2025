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
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.LimelightHelper.PoseEstimate;
import frc.robot.utilities.constants.FieldConstants;

public class LimelightSubsystem extends SubsystemBase {
    private static Limelight leftElevator = new Limelight("left-elevator");
    private static Limelight rightElevator = new Limelight("right-elevator");
    private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public LimelightSubsystem() {}

    @Override
    public void periodic() {
        leftElevator.update();
        rightElevator.update();

        Pose2d pose = leftElevator.getCurrentBotPoseBlue();
        drivetrain.addVisionMeasurement(pose, 0);
    }

    public static Limelight getLeftElevator() {
        return leftElevator;
    }

    public static Limelight getRightElevator() {
        return rightElevator;
    }
}