// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.SwerveController;
import frc.robot.subsystems.AlgeaGroundSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.constants.Constants;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  CoralSubsystem coralSubsystem = new CoralSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(); // Create an instance
  AlgeaGroundSubsystem algeaGroundSubsystem = new AlgeaGroundSubsystem(); // Create an instance

  private final XboxController DriverController, OperatorController, TestingController;

  private final SendableChooser<Command> autonomousChooser;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    DriverController = new XboxController(Constants.DriverConstants.driverControllerPort);
    OperatorController = new XboxController(Constants.DriverConstants.operatorControllerPort);
    TestingController = new XboxController(Constants.DriverConstants.tesingControllerPort);

    autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser",autonomousChooser);
    RobotController.setBrownoutVoltage(6.75);

    swerveSubsystem.setDefaultCommand(new SwerveController(
      swerveSubsystem,  
      () -> -DriverController.getLeftY(),
      () -> -DriverController.getLeftX(), 
      () -> DriverController.getRightX()
    ));

    configureDriverController();
    configureOperatorController();
    configureTestingController();
  }

  public void configureDriverController() {
    new Trigger(() -> DriverController.getBackButton()).onTrue(new InstantCommand(() -> swerveSubsystem.resetYaw(Rotation2d.fromDegrees(0.0))));
    new Trigger(() -> DriverController.getStartButton()).onTrue(new InstantCommand(() -> swerveSubsystem.switchDriveMode()));
   // new Trigger(() -> DriverController.getRightBumperButton()).onTrue(new InstantCommand(() -> swerveSubsystem.switchSlowMode()));



    new Trigger(() -> DriverController.getAButton())
    .onTrue(new InstantCommand(() -> coralSubsystem.Coralspin(0.5), coralSubsystem)); // Runs motors at 50%

new Trigger(() -> DriverController.getAButton())
    .onFalse(new InstantCommand(() -> coralSubsystem.Coralspin(0.0), coralSubsystem)); // Stops motors


    new Trigger(() -> DriverController.getLeftBumperButton())
    .onTrue(new InstantCommand(() -> algeaGroundSubsystem.Algeaspin(0.5), algeaGroundSubsystem)); // Runs motors at 50%

    new Trigger(() -> DriverController.getLeftBumperButton())
    .onFalse(new InstantCommand(() -> algeaGroundSubsystem.AlgeaSpinstop(), algeaGroundSubsystem));
    
    
    
    
    
    new Trigger(() -> DriverController.getRightBumperButton())
    .onTrue(new InstantCommand(() -> algeaGroundSubsystem.AlgeaPivot(0.5), algeaGroundSubsystem)); // Runs motors at 50%

    new Trigger(() -> DriverController.getRightBumperButton())
    .onFalse(new InstantCommand(() -> algeaGroundSubsystem.AlgeaPivotstop(0.0), algeaGroundSubsystem));// Stops motors

    new Trigger(() -> DriverController.getBButton())
    .whileTrue(new InstantCommand(() -> elevatorSubsystem.runElevatorUp(0.2), elevatorSubsystem));// Stops motors

    new Trigger(() -> DriverController.getBButton())
    .onFalse(new InstantCommand(() -> elevatorSubsystem.runElevatorUp(0.0), elevatorSubsystem));// Stops motors
  }


  public void configureOperatorController() {}

  public void configureTestingController() {
    //new Trigger(() -> TestingController.getAButton()).onTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //new Trigger(() -> TestingController.getBButton()).onTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //new Trigger(() -> TestingController.getXButton()).onTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //new Trigger(() -> TestingController.getYButton()).onTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}