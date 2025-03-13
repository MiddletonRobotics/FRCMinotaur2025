// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import frc.robot.commands.AlgeaGroundCommands;
import frc.robot.commands.CoralSpinnerCommands;
import frc.robot.commands.DealgeafierCommands;
import frc.robot.commands.ElevatorCommands;
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
    private final CommandXboxController manualController = new CommandXboxController(2);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final DealgeafierSubsystem dealgeafierSubsystem = new DealgeafierSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ProcessorSubsystem processorSubsystem = new ProcessorSubsystem();

    public RobotContainer() {
        autonomousChooser = AutoBuilder.buildAutoChooser();

        new EventTrigger("Dealgeafier Reef").onTrue(DealgeafierCommands.runPivotToPosition(dealgeafierSubsystem, PivotingState.REEF));
        new EventTrigger("Run Dealgeafier").onTrue(dealgeafierSubsystem.run(() -> dealgeafierSubsystem.startRolling(1)));
        new EventTrigger("Stop Dealgeafier").onTrue(dealgeafierSubsystem.run(() -> dealgeafierSubsystem.stopRolling()));

        SmartDashboard.putData("Autonomous Chooser", autonomousChooser);

        configureDriverBindings();
        configureOperatorBindings();
        configureManualBindings();
    }

    public void onTeleopInit() {
        dealgeafierSubsystem.setNeutralModes(IdleMode.kBrake);
        dealgeafierSubsystem.stopRolling();
    }

    public void onDisabled() {
        dealgeafierSubsystem.setNeutralModes(IdleMode.kCoast);
    }

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

        // Run SysId routines when holding back/start and X/Y. Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureOperatorBindings() {   
        operatorController.leftBumper().onTrue(DealgeafierCommands.intakeUntilBroken(dealgeafierSubsystem, 1));
        operatorController.rightBumper().onTrue(DealgeafierCommands.shootAlgea(dealgeafierSubsystem, 1));

        operatorController.a().onTrue(CoralSpinnerCommands.funnelIntakingUntilBroken(coralSubsystem, null));
        operatorController.b().onTrue(CoralSpinnerCommands.scoreCoral(coralSubsystem, null));
        operatorController.x().onTrue(AlgeaGroundCommands.intakeAlgea(processorSubsystem, GroundPivotingState.GROUND));
        operatorController.y().onTrue(AlgeaGroundCommands.spitOutBall(processorSubsystem));
    }

    private void configureManualBindings() {
        manualController.a().whileTrue(new InstantCommand(() -> processorSubsystem.startGroundPivot(0.15)));
        manualController.a().whileFalse(new InstantCommand(() -> processorSubsystem.stopGroundPivot()));

        manualController.b().whileTrue(new InstantCommand(() -> processorSubsystem.startGroundPivot(-0.15)));
        manualController.b().whileFalse(new InstantCommand(() -> processorSubsystem.stopGroundPivot()));

        manualController.x().whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(-0.7)));
        manualController.x().whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        manualController.y().whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(0.4)));
        manualController.y().whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        manualController.povUp().whileTrue(new RunCommand(() -> elevatorSubsystem.runElevatorUp(0.4)));
        manualController.povUp().onFalse(new InstantCommand(() -> elevatorSubsystem.runElevatorUp(0.0)));

        manualController.povDown().whileTrue(new RunCommand(() -> elevatorSubsystem.runElevatorDown(-0.05)));
        manualController.povDown().onFalse(new InstantCommand(() -> elevatorSubsystem.runElevatorDown(0.0)));

        manualController.leftBumper().whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(1)));
        manualController.leftBumper().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        manualController.rightBumper().whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(-1)));
        manualController.rightBumper().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        manualController.rightTrigger(0.5).whileTrue(new InstantCommand(() -> dealgeafierSubsystem.startRolling(1)));
        manualController.rightTrigger(0.5).whileFalse(new InstantCommand(() -> dealgeafierSubsystem.stopRolling()));

        manualController.leftTrigger(0.5).whileTrue(new InstantCommand(() -> dealgeafierSubsystem.startRolling(-1)));
        manualController.leftTrigger(0.5).whileFalse(new InstantCommand(() -> dealgeafierSubsystem.stopRolling()));
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}
