// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.constants.Constants;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final CommandXboxController DriverController;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    DriverController = new CommandXboxController(Constants.DriverConstants.driverControllerPort);

    RobotController.setBrownoutVoltage(6.0);

    swerveSubsystem.setDefaultCommand(new SwerveController(
      swerveSubsystem,  
      () -> -DriverController.getLeftY(),
      () -> -DriverController.getLeftX(), 
      () -> DriverController.getRightX(),
      () -> false
    ));

    configureBindings();
  }

  private void configureBindings() {
    DriverController.a().toggleOnTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
