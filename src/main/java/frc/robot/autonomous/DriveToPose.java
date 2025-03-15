package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPose extends Command {
  
  CommandSwerveDrivetrain drivetrain;
  Pose2d targetPose;
  PIDController driveController;

  private SwerveRequest.ApplyRobotSpeeds driveChassisSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    addRequirements(drivetrain);
    setName("DriveToPose");

    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.driveController = new PIDController(0.1, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationValue = new Translation2d(driveController.calculate(drivetrain.getState().Pose.getX(), targetPose.getX()), driveController.calculate(drivetrain.getState().Pose.getY(), targetPose.getY()));
    drivetrain.applyRequest(() -> driveChassisSpeeds.withSpeeds(new ChassisSpeeds(translationValue.getX(), translationValue.getY(), targetPose.getRotation().getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
