package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utilities.LimelightHelper;

public class AutoAlign extends Command {

  CommandSwerveDrivetrain drivetrain;
  Rotation2d targetAngle;
  double targetY;
  LimelightSubsystem limelightSubsystem;

  PIDController xController;
  PIDController yController;
  PIDController rotationController;

  private SwerveRequest.ApplyRobotSpeeds driveChassisSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  public AutoAlign(CommandSwerveDrivetrain drivetrain, Rotation2d targetAngle, double targetY, LimelightSubsystem limelightSubsystem) {
    addRequirements(drivetrain  );
    setName("AutoAlign");

    this.drivetrain = drivetrain;
    this.targetAngle = targetAngle;
    this.targetY = targetY;
    this.limelightSubsystem = limelightSubsystem;

    xController = new PIDController(0.1, 0.0, 0.0);
    yController = new PIDController(0.1, 0.0, 0.0);;
    rotationController = new PIDController(0.1, 0.0, 0.0);;
    rotationController.enableContinuousInput(-180.0, 180.0);
    xController.setTolerance(0.5);
    yController.setTolerance(0.5);
    rotationController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String choosenLimelight = limelightSubsystem.chooseLimelight();

    double xValue = LimelightHelper.getTV(choosenLimelight) ? -yController.calculate(LimelightHelper.getTY(choosenLimelight), targetY) : 0;
    double yValue = LimelightHelper.getTV(choosenLimelight) ? xController.calculate(LimelightHelper.getTX(choosenLimelight), 0.0) : 0;
    double rotationValue = rotationController.calculate(drivetrain.getPigeon2().getYaw().getValue().in(Degrees), targetAngle.getDegrees());

    drivetrain.applyRequest(() -> driveChassisSpeeds.withSpeeds(new ChassisSpeeds(xValue, yValue, rotationValue)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}