// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CoralSpinnerCommands;
import frc.robot.commands.DealgeafierCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgeaElevatorSubsystem;
//import frc.robot.subsystems.AlgeaElevatorSubsystem.RollUntilLimitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgeaElevatorSubsystem.PivotingState;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autonomousChooser;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final AlgeaElevatorSubsystem algeaElevatorSubsystem = new AlgeaElevatorSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public RobotContainer() {
        autonomousChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous Chooser", autonomousChooser);

        configureBindings();
    }

    public void onInit() {
        algeaElevatorSubsystem.stopRolling();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        operatorController.a().whileTrue(new InstantCommand(() -> algeaElevatorSubsystem.startRolling(1)));
        operatorController.a().whileFalse(new InstantCommand(() -> algeaElevatorSubsystem.stopRolling()));

        operatorController.b().whileTrue(new InstantCommand(() -> algeaElevatorSubsystem.startRolling(-1)));
        operatorController.b().whileFalse(new InstantCommand(() -> algeaElevatorSubsystem.stopRolling()));


        //operatorController.x().whileTrue(new InstantCommand(() -> coralSubsystem.Coralspin(-0.7)));
        //operatorController.x().whileFalse(new InstantCommand(() -> coralSubsystem.CoralSpinstop()));

        operatorController.rightBumper().whileTrue(new RunCommand(() -> elevatorSubsystem.runElevatorUp(0.4)));
        operatorController.rightBumper().onFalse(new InstantCommand(() -> elevatorSubsystem.runElevatorUp(0.0)));

        operatorController.leftBumper().whileTrue(new RunCommand(() -> elevatorSubsystem.runElevatorDown(-0.05)));
        operatorController.leftBumper().onFalse(new InstantCommand(() -> elevatorSubsystem.runElevatorDown(0.0)));

        operatorController.x().onTrue(DealgeafierCommands.runPivotToPosition(algeaElevatorSubsystem, PivotingState.REEF));
        operatorController.y().onTrue(DealgeafierCommands.runPivotToPosition(algeaElevatorSubsystem, PivotingState.STORED));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return autonomousChooser.getSelected();
        return Commands.print("L no auto");
    }
}
