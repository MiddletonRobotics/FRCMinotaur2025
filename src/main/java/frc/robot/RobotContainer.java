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
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.ProcessorCommands;
import frc.robot.commands.SwervePosePID;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DealgeafierCommands;
import frc.robot.commands.Elevator2Commands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.PrepareDealgeafication;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem2;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.DealgeafierSubsystem.PivotingState;
import frc.robot.subsystems.ProcessorSubsystem.GroundPivotingState;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public enum DrivingState {
        ELEVATOR_UP(0.2),
        SLOWMODE(0.3),
        NORMAL(0.8);

        private final double position;

        private DrivingState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public DrivingState drivingState = DrivingState.NORMAL;

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autonomousChooser;

    private int driverControllerPort = 0;
    private int operatorControllerPort = 1;

    private final CommandXboxController driverController = new CommandXboxController(driverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(operatorControllerPort);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public CoralSubsystem coralSubsystem = new CoralSubsystem();
    public final DealgeafierSubsystem dealgeafierSubsystem = new DealgeafierSubsystem();
    public final ElevatorSubsystem2 elevatorSubsystem2 = new ElevatorSubsystem2();
    public final ProcessorSubsystem processorSubsystem = new ProcessorSubsystem();
    //private final Limelight leftElevatorLL = new Limelight(drivetrain, "limelight-left");
    //private final Limelight rightElevatorLL = new Limelight(drivetrain, "limelight-right");

    public RobotContainer() {
        NamedCommands.registerCommand("StowElevator", Elevator2Commands.runElevatorToStow(elevatorSubsystem2));
        NamedCommands.registerCommand("PrepareBarge", Elevator2Commands.runElevatorToBarge(elevatorSubsystem2));
        NamedCommands.registerCommand("ProcessorStore", ProcessorCommands.startPivotToStored(processorSubsystem));
        NamedCommands.registerCommand("PrepareL4", Elevator2Commands.runElevatorToL4(elevatorSubsystem2));
        NamedCommands.registerCommand("PrepareL3", Elevator2Commands.runElevatorToL3(elevatorSubsystem2));
        NamedCommands.registerCommand("PrepareL1", Elevator2Commands.runElevatorToL1(elevatorSubsystem2));
        NamedCommands.registerCommand("ScoreCoral", CoralCommands.scoreCoral(coralSubsystem));
        NamedCommands.registerCommand("IndexCoral", CoralCommands.funnelIntakingUntilBroken(coralSubsystem));
        NamedCommands.registerCommand("PrepareDealgification", new PrepareDealgeafication(dealgeafierSubsystem, elevatorSubsystem2));
        NamedCommands.registerCommand("AlgeaBarge", DealgeafierCommands.shootAlgeaSensorless(dealgeafierSubsystem));
        NamedCommands.registerCommand("AlgeaStart", DealgeafierCommands.runPivotToStart(dealgeafierSubsystem));
        NamedCommands.registerCommand("AlgeaTilt", DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem));

        autonomousChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        //SmartDashboard.putNumber("RoboRIO CPU", RobotController.get)

        //configureDriverBindings();
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

        /*
         
    private void configureDriverBindings() {
        // Note that X is defined as forward according to WPILib convention, and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(limiterX.calculate(-driverController.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(limiterY.calculate(-driverController.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(limiterRotate.calculate(-driverController.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.x().whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(-1)));
        driverController.x().whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        driverController.y().whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(0.4)));
        driverController.y().whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        driverController.leftTrigger(0.5).whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(0.5)));
        driverController.leftTrigger(0.5).onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.rightTrigger(0.5).whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(-0.2)));
        driverController.rightTrigger(0.5).onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.leftBumper().whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(1)));
        driverController.leftBumper().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        driverController.rightBumper().whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(-1)));
        driverController.rightBumper().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        driverController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    */

    public void configureOperatorBindings() {   
        operatorController.y().onTrue(ProcessorCommands.startPivotToStored(processorSubsystem));
        operatorController.x().onTrue(ProcessorCommands.startPivotToGround(processorSubsystem));

        operatorController.b().onTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(1)));
        operatorController.b().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));
        operatorController.a().onTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(-1)));
        operatorController.a().whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        operatorController.povRight().onTrue(DealgeafierCommands.runPivotToStart(dealgeafierSubsystem));
        operatorController.povUp().onTrue(DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem));
        operatorController.povLeft().onTrue(DealgeafierCommands.runPivotToReef(dealgeafierSubsystem));
        operatorController.povDown().onTrue(DealgeafierCommands.runPivotToGround(dealgeafierSubsystem));

        operatorController.rightTrigger().onTrue(DealgeafierCommands.intakeUntilBroken(dealgeafierSubsystem));
        operatorController.rightBumper().onTrue(DealgeafierCommands.shootAlgea(dealgeafierSubsystem));

        operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.5).onTrue(new InstantCommand(() -> elevatorSubsystem2.incrementElevatorState()));
        operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.5).onTrue(new InstantCommand(() -> elevatorSubsystem2.decrementElevatorState()));
    }

    public void configureTestingBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-driverController.getLeftY() * MaxSpeed) * drivingState.getPosition()) // Drive forward with negative Y (forward)
                    .withVelocityY((-driverController.getLeftX() * MaxSpeed) * drivingState.getPosition()) // Drive left with negative X (left)
                    .withRotationalRate((-driverController.getRightX() * MaxAngularRate) * drivingState.getPosition()) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.leftStick().onTrue(new SwervePosePID(drivetrain, true));
        driverController.rightStick().onTrue(new SwervePosePID(drivetrain, false));

        driverController.a().onTrue(Elevator2Commands.runElevatorToPosition(elevatorSubsystem2));
        driverController.b().onTrue(Elevator2Commands.runElevatorToStow(elevatorSubsystem2));

        driverController.povLeft().whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(0.5)));
        driverController.povLeft().onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.povRight().whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(-0.2)));
        driverController.povRight().onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.leftTrigger().onTrue(CoralCommands.funnelIntakingUntilBroken(coralSubsystem));
        driverController.leftBumper().onTrue(CoralCommands.scoreCoral(coralSubsystem));
        driverController.rightTrigger(0.5).onTrue(new ConditionalCommand(
            new InstantCommand(() -> this.drivingState = DrivingState.SLOWMODE), 
            new InstantCommand(() -> this.drivingState = DrivingState.NORMAL), 
            () -> this.drivingState == DrivingState.NORMAL
        ));

        driverController.back().and(driverController.rightBumper()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}
