// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utilities.LimelightHelper;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  //private Pose2d leftElevatorPose;
  //private Pose2d rightElevatorPose;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    m_robotContainer.onDisabled();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    //leftElevatorPose = LimelightSubsystem.getLeftElevator().getCurrentBotPoseBlue();
    //rightElevatorPose = LimelightSubsystem.getRightElevator().getCurrentBotPoseBlue();

    //m_robotContainer.drivetrain.setStateStdDevs(VecBuilder.fill(.7, .7, 9999999));
    //m_robotContainer.drivetrain.addVisionMeasurement(leftElevatorPose, kDefaultPeriod);
    //m_robotContainer.drivetrain.addVisionMeasurement(rightElevatorPose, kDefaultPeriod);
  }

  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.onTeleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
  }
}
