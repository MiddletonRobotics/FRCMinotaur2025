// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ProcessorCommands;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DealgeafierCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.PrepareDealgeafication;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;
//import frc.robot.subsystems.AlgeaElevatorSubsystem.RollUntilLimitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DealgeafierSubsystem.PivotingState;
import frc.robot.subsystems.ProcessorSubsystem.GroundPivotingState;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autonomousChooser;

    private int driverControllerPort = 0;
    private int operatorControllerPort = 1;
    private final CommandXboxController driverController = new CommandXboxController(driverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(operatorControllerPort);
    private final CommandXboxController manualController = new CommandXboxController(2);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final DealgeafierSubsystem dealgeafierSubsystem = new DealgeafierSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ProcessorSubsystem processorSubsystem = new ProcessorSubsystem();

    private SlewRateLimiter limiter = new SlewRateLimiter(2.9);

    public RobotContainer() {
        NamedCommands.registerCommand("StowElevator", ElevatorCommands.autoStowElevator(elevatorSubsystem));
        NamedCommands.registerCommand("PrepareL4", ElevatorCommands.autoPrepareL4(elevatorSubsystem));
        NamedCommands.registerCommand("PrepareL3", ElevatorCommands.autoPrepareL3(elevatorSubsystem));
        NamedCommands.registerCommand("PrepareL1", ElevatorCommands.autoPrepareL1(elevatorSubsystem));
        NamedCommands.registerCommand("ScoreCoral", CoralCommands.scoreCoral(coralSubsystem));
        NamedCommands.registerCommand("IndexCoral", CoralCommands.funnelIntakingUntilBroken(coralSubsystem));
        NamedCommands.registerCommand("PrepareDealgification", new PrepareDealgeafication(dealgeafierSubsystem, elevatorSubsystem));
        NamedCommands.registerCommand("AlgeaBarge", DealgeafierCommands.shootAlgeaSensorless(dealgeafierSubsystem));
        NamedCommands.registerCommand("AlgeaStart", DealgeafierCommands.runPivotToStart(dealgeafierSubsystem));
        NamedCommands.registerCommand("AlgeaTilt", DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem));

        autonomousChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

        configureDriverBindings();
        configureOperatorBindings();
        configureTestingBindings();

        PathfindingCommand.warmupCommand().schedule();
    }

    public void onTeleopInit() {
        dealgeafierSubsystem.setNeutralModes(IdleMode.kBrake);
        processorSubsystem.setNeutralModes(IdleMode.kBrake);
        dealgeafierSubsystem.stopRolling();
    }

    public void onDisabled() {
        dealgeafierSubsystem.setNeutralModes(IdleMode.kCoast);
        processorSubsystem.setNeutralModes(IdleMode.kCoast);
    }

    /* 

    public void configureDriverBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));
        driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.x().onTrue(ElevatorCommands.runElevatorwithIntegratedController(elevatorSubsystem));

        driverController.leftTrigger().onTrue(CoralCommands.funnelIntakingUntilBroken(coralSubsystem));
        driverController.leftBumper().onTrue(CoralCommands.scoreCoral(coralSubsystem));
        driverController.rightTrigger().onTrue(DealgeafierCommands.intakeUntilBroken(dealgeafierSubsystem));
        driverController.rightBumper().onTrue(DealgeafierCommands.shootAlgea(dealgeafierSubsystem));
    }

    public void configureOperatorBindings() {
        operatorController.leftBumper().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L1)));
        operatorController.leftTrigger(0.5).onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L2)));
        operatorController.rightBumper().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L3)));
        operatorController.rightTrigger(0.5).onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L4)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.BARGE)));
        operatorController.povRight().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L2)));
        operatorController.povUp().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L3)));
        operatorController.x().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.STOW)));

        operatorController.y().onTrue(new ConditionalCommand(
            DealgeafierCommands.runPivotToStored(dealgeafierSubsystem),
            DealgeafierCommands.runPivotToReef(dealgeafierSubsystem).andThen(DealgeafierCommands.intakeUntilBroken(dealgeafierSubsystem)),
            () -> dealgeafierSubsystem.getPivotingState() == PivotingState.REEF || dealgeafierSubsystem.getLimitSwitch()
        ));

        operatorController.start().onTrue(DealgeafierCommands.shootAlgea(dealgeafierSubsystem));
        operatorController.a().onTrue(ProcessorCommands.intakeAlgea(processorSubsystem, GroundPivotingState.GROUND));
        operatorController.b().onTrue(ProcessorCommands.spitOutBall(processorSubsystem));
    }
        */
         
    private void configureDriverBindings() {
        // Note that X is defined as forward according to WPILib convention, and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.x().whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(-1)));
        driverController.x().whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        driverController.y().whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(0.4)));
        driverController.y().whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        driverController.leftTrigger(0.5).whileTrue(new RunCommand(() -> elevatorSubsystem.runElevatorUp(-0.77)));
        driverController.leftTrigger(0.5).onFalse(new InstantCommand(() -> elevatorSubsystem.runElevatorUp(0.0)));

        driverController.rightTrigger(0.5).whileTrue(new RunCommand(() -> elevatorSubsystem.runElevatorDown(0.3)));
        driverController.rightTrigger(0.5).onFalse(new InstantCommand(() -> elevatorSubsystem.runElevatorDown(0.0)));

        driverController.leftBumper().whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(1)));
        driverController.leftBumper().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        driverController.rightBumper().whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(-1)));
        driverController.rightBumper().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        driverController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureOperatorBindings() {   
        operatorController.y().whileTrue(new InstantCommand(() -> processorSubsystem.startGroundPivot(0.2)));
        operatorController.y().whileFalse(new InstantCommand(() -> processorSubsystem.startGroundPivot(0.0)));

        operatorController.x().whileTrue(new InstantCommand(() -> processorSubsystem.startGroundPivot(-0.125)));
        operatorController.x().whileFalse(new InstantCommand(() -> processorSubsystem.stopGroundPivot()));

        operatorController.povRight().whileTrue(new RunCommand(() -> dealgeafierSubsystem.startPivot(-0.23)));
        operatorController.povRight().onFalse(new InstantCommand(() -> dealgeafierSubsystem.stopPivot()));

        operatorController.povLeft().whileTrue(new RunCommand(() -> dealgeafierSubsystem.startPivot(0.23)));
        operatorController.povLeft().onFalse(new InstantCommand(() -> dealgeafierSubsystem.stopPivot()));

        operatorController.rightTrigger(0.5).whileTrue(new InstantCommand(() -> dealgeafierSubsystem.startRolling(1)));
        operatorController.rightTrigger(0.5).whileFalse(new InstantCommand(() -> dealgeafierSubsystem.stopRolling()));

        operatorController.leftTrigger(0.5).whileTrue(new InstantCommand(() -> dealgeafierSubsystem.startRolling(-1)));
        operatorController.leftTrigger(0.5).whileFalse(new InstantCommand(() -> dealgeafierSubsystem.stopRolling()));
    }

    public void configureTestingBindings() {
        manualController.a().onTrue(DealgeafierCommands.runPivotToStored(dealgeafierSubsystem));
        manualController.b().onTrue(DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem));
        manualController.x().onTrue(DealgeafierCommands.runPivotToReef(dealgeafierSubsystem));
        manualController.y().onTrue(DealgeafierCommands.runPivotToStart(dealgeafierSubsystem));

        manualController.povDown().onTrue(ElevatorCommands.runElevatorwithIntegratedController(elevatorSubsystem));

        operatorController.leftBumper().whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L1)));
        operatorController.leftTrigger(0.5).whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L2)));
        operatorController.rightBumper().whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L3)));
        operatorController.rightTrigger(0.5).whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L4)));
        operatorController.povLeft().whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.BARGE)));
        operatorController.povRight().whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L2)));
        operatorController.povUp().whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L3)));
        operatorController.leftStick().whileTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.STOW)));
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}
