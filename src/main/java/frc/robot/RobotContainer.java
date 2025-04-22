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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ProcessorCommands;
import frc.robot.commands.RetractElevatorSafe;
import frc.robot.commands.ShootBarge;
import frc.robot.commands.ShootBargeNoRetraction;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DealgeafierCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.LEDCommands;
import frc.robot.commands.PrepareDealgeafication;
import frc.robot.commands.PrepareDealgeaficationL3;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ProcessorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.LimelightConstants.REEFS;

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
    private final XboxController driverControllerHID = driverController.getHID();
    private final CommandXboxController operatorController = new CommandXboxController(operatorControllerPort);
    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.system();

    public CoralSubsystem coralSubsystem = new CoralSubsystem();
    public final DealgeafierSubsystem dealgeafierSubsystem = new DealgeafierSubsystem();
    public final ElevatorSubsystem elevatorSubsystem2 = new ElevatorSubsystem();
    public final ProcessorSubsystem processorSubsystem = new ProcessorSubsystem();
    //public final Limelight limelights = Limelight.system();
    private LEDSubsystem ledSubsystem = new LEDSubsystem();
    private boolean isManual = false;
    //private final Limelight leftElevatorLL = new Limelight(drivetrain, "limelight-left");
    //private final Limelight rightElevatorLL = new Limelight(drivetrain, "limelight-right");

    public RobotContainer() {
        NamedCommands.registerCommand("StowElevator", ElevatorCommands.runElevatorToStow(elevatorSubsystem2));
        NamedCommands.registerCommand("ZeroElevator",ElevatorCommands.runElevatorToReset(elevatorSubsystem2));
        NamedCommands.registerCommand("PrepareBarge", ElevatorCommands.runElevatorToBarge(elevatorSubsystem2));
        NamedCommands.registerCommand("ProcessorStore", ProcessorCommands.startPivotToStored(processorSubsystem));
        NamedCommands.registerCommand("PrepareL4", ElevatorCommands.runElevatorToL4(elevatorSubsystem2));
        NamedCommands.registerCommand("PrepareL3", ElevatorCommands.runElevatorToL3(elevatorSubsystem2));
        NamedCommands.registerCommand("PrepareL1", ElevatorCommands.runElevatorToL1(elevatorSubsystem2));
        NamedCommands.registerCommand("ScoreCoral", CoralCommands.scoreCoral(coralSubsystem, ledSubsystem));
        NamedCommands.registerCommand("IndexCoral", CoralCommands.funnelIntakingUntilBroken(coralSubsystem, elevatorSubsystem2, ledSubsystem));
        NamedCommands.registerCommand("PrepareDealgification", new PrepareDealgeafication(dealgeafierSubsystem, elevatorSubsystem2));
        NamedCommands.registerCommand("PrepareDealgificationL3", new PrepareDealgeaficationL3(dealgeafierSubsystem, elevatorSubsystem2));
        NamedCommands.registerCommand("AlgeaBarge", DealgeafierCommands.shootAlgeaSensorless(dealgeafierSubsystem));
        NamedCommands.registerCommand("AlgeaStart", DealgeafierCommands.runPivotToStart(dealgeafierSubsystem));
        NamedCommands.registerCommand("AlgeaTilt", DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem));
        NamedCommands.registerCommand("ShootBargeNoRetraction", new ShootBargeNoRetraction(elevatorSubsystem2, dealgeafierSubsystem, ledSubsystem));
        NamedCommands.registerCommand("ShootBarge", new ShootBarge(elevatorSubsystem2, dealgeafierSubsystem, ledSubsystem));
        NamedCommands.registerCommand("RetractElevatorSafe", new RetractElevatorSafe(elevatorSubsystem2, dealgeafierSubsystem, ledSubsystem));

        new EventTrigger("ETPrepareDealgification").whileTrue(new PrepareDealgeafication(dealgeafierSubsystem, elevatorSubsystem2));
        new EventTrigger("ETStoreDealgeafier").whileTrue(DealgeafierCommands.runPivotToStored(dealgeafierSubsystem));
        new EventTrigger("ETReefDealgeafier").whileTrue(DealgeafierCommands.runPivotToReef(dealgeafierSubsystem));

        autonomousChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
        //SmartDashboard.putNumber("RoboRIO CPU", RobotController.get)

        //configureDriverBindings();
        configureOperatorBindings();
        configureTestingBindings();

        PathfindingCommand.warmupCommand().schedule();
    }

    public void onTeleopInit() {
        dealgeafierSubsystem.setNeutralModes(IdleMode.kBrake);
        processorSubsystem.setNeutralModes(IdleMode.kBrake);
        ledSubsystem.setPattern(Constants.DriverConstants.DEF_PATTERN);
        dealgeafierSubsystem.stopRolling();
    }

    public void onAutonomousInit() {
        ledSubsystem.setPattern(Constants.DriverConstants.DEF_PATTERN);
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
        operatorController.y().and(() -> !isManual).onTrue(ProcessorCommands.startPivotToIntaken(processorSubsystem));
        operatorController.x().and(() -> !isManual).onTrue(ProcessorCommands.startPivotToGround(processorSubsystem));
        operatorController.start().and(() -> !isManual).onTrue(ProcessorCommands.startPivotToStored                                                                                                                                                                                 (processorSubsystem));

        operatorController.b().and(() -> !isManual).onTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(0.35)));
        operatorController.b().and(() -> !isManual).whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));
        operatorController.a().and(() -> !isManual).onTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(-0.6)));
        operatorController.a().and(() -> !isManual).whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        operatorController.povRight().and(() -> !isManual).onTrue(DealgeafierCommands.runPivotToStart(dealgeafierSubsystem));
        operatorController.povUp().and(() -> !isManual).onTrue(DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem));
        operatorController.povLeft().and(() -> !isManual).onTrue(DealgeafierCommands.runPivotToReef(dealgeafierSubsystem));
        operatorController.povDown().and(() -> !isManual).onTrue(DealgeafierCommands.runPivotToGround(dealgeafierSubsystem));

        operatorController.rightTrigger().and(() -> !isManual).onTrue(DealgeafierCommands.intakeUntilBroken(dealgeafierSubsystem));
        operatorController.rightBumper().and(() -> !isManual).onTrue(DealgeafierCommands.shootAlgea(dealgeafierSubsystem, ledSubsystem));

        operatorController.leftBumper().and(() -> !isManual).whileTrue(new RunCommand(() -> dealgeafierSubsystem.startPivot(-0.23)));
        operatorController.leftBumper().and(() -> !isManual).onFalse(new InstantCommand(() -> dealgeafierSubsystem.stopPivot()));

        operatorController.leftTrigger(0.5).and(() -> !isManual).whileTrue(new RunCommand(() -> dealgeafierSubsystem.startPivot(0.23)));
        operatorController.leftTrigger(0.5).and(() -> !isManual).onFalse(new InstantCommand(() -> dealgeafierSubsystem.stopPivot()));

        operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.35).and(() -> !isManual).onTrue(new InstantCommand(() -> elevatorSubsystem2.incrementElevatorState()));
        operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.35).and(() -> !isManual).onTrue(new InstantCommand(() -> elevatorSubsystem2.decrementElevatorState()));

        operatorController.y().and(() -> isManual).whileTrue(new InstantCommand(() -> processorSubsystem.startGroundPivot(0.2)));
        operatorController.y().and(() -> isManual).whileFalse(new InstantCommand(() -> processorSubsystem.startGroundPivot(0.0)));

        operatorController.x().and(() -> isManual).whileTrue(new InstantCommand(() -> processorSubsystem.startGroundPivot(-0.125)));
        operatorController.x().and(() -> isManual).whileFalse(new InstantCommand(() -> processorSubsystem.stopGroundPivot()));

        operatorController.povRight().and(() -> isManual).whileTrue(new RunCommand(() -> dealgeafierSubsystem.startPivot(-0.23)));
        operatorController.povRight().and(() -> isManual).onFalse(new InstantCommand(() -> dealgeafierSubsystem.stopPivot()));

        operatorController.povLeft().and(() -> isManual).whileTrue(new RunCommand(() -> dealgeafierSubsystem.startPivot(0.23)));
        operatorController.povLeft().and(() -> isManual).onFalse(new InstantCommand(() -> dealgeafierSubsystem.stopPivot()));
   
        operatorController.rightTrigger(0.5).and(() -> isManual).whileTrue(new InstantCommand(() -> dealgeafierSubsystem.startRolling(1)));
        operatorController.rightTrigger(0.5).and(() -> isManual).whileFalse(new InstantCommand(() -> dealgeafierSubsystem.stopRolling()));

        operatorController.leftTrigger(0.5).and(() -> isManual).whileTrue(new InstantCommand(() -> dealgeafierSubsystem.startRolling(-1)));
        operatorController.leftTrigger(0.5).and(() -> isManual).whileFalse(new InstantCommand(() -> dealgeafierSubsystem.stopRolling()));
    }

    public void configureTestingBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-driverControllerHID.getLeftY() * MaxSpeed) * drivingState.getPosition()) // Drive forward with negative Y (forward)
                    .withVelocityY((-driverControllerHID.getLeftX() * MaxSpeed) * drivingState.getPosition()) // Drive left with negative X (left)
                    .withRotationalRate((-driverControllerHID.getRightX() * MaxAngularRate) * drivingState.getPosition()) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().and(() -> !isManual).onTrue(ElevatorCommands.runElevatorToPosition(elevatorSubsystem2));
        driverController.b().and(() -> !isManual).onTrue(ElevatorCommands.runElevatorToStow(elevatorSubsystem2));
        driverController.y().and(() -> !isManual).onTrue(ElevatorCommands.runElevatorToBarge(elevatorSubsystem2));
        driverController.x().onTrue(new InstantCommand(() -> isManual = !isManual));

        driverController.povUp().and(() -> !isManual).whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(0.3)));
        driverController.povUp().and(() -> !isManual).onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.povRight().and(() -> !isManual).onTrue(drivetrain.alignToRightReef(Constants.LimelightConstants.REEFS.RIGHT));

        driverController.povDown().and(() -> !isManual).whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(-0.2)));
        driverController.povDown().and(() -> !isManual).onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.povLeft().and(() -> !isManual).onTrue(drivetrain.alignToRightReef(Constants.LimelightConstants.REEFS.LEFT));

        driverController.leftTrigger().and(() -> !isManual).onTrue(CoralCommands.funnelIntakingUntilBroken(coralSubsystem, elevatorSubsystem2, ledSubsystem));
        driverController.leftBumper().and(() -> !isManual).onTrue(CoralCommands.scoreCoral(coralSubsystem, ledSubsystem));
        driverController.rightBumper().and(() -> !isManual).onTrue(new ConditionalCommand(
            new InstantCommand(() -> this.drivingState = DrivingState.SLOWMODE), 
            new InstantCommand(() -> this.drivingState = DrivingState.NORMAL), 
            () -> this.drivingState == DrivingState.NORMAL
        ));

        driverController.rightTrigger(0.5).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancel(drivetrain.alignToRightReef(null))));

        driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driverController.start().and(() -> !isManual).onTrue(new InstantCommand(() -> elevatorSubsystem2.resetEncoders()));

        driverController.a().and(() -> isManual).whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().and(() -> isManual).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.x().and(() -> isManual).whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(-1)));
        driverController.x().and(() -> isManual).whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        driverController.y().and(() -> isManual).whileTrue(new InstantCommand(() -> coralSubsystem.spinCoral(0.4)));
        driverController.y().and(() -> isManual).whileFalse(new InstantCommand(() -> coralSubsystem.stopCoral()));

        driverController.leftTrigger(0.5).and(() -> isManual).whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(-0.9)));
        driverController.leftTrigger(0.5).and(() -> isManual).onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.rightTrigger(0.5).and(() -> isManual).whileTrue(new RunCommand(() -> elevatorSubsystem2.setSpeed(0.2)));
        driverController.rightTrigger(0.5).and(() -> isManual).onFalse(new InstantCommand(() -> elevatorSubsystem2.setSpeed(0.0)));

        driverController.leftBumper().and(() -> isManual).whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(0.25)));
        driverController.leftBumper().and(() -> isManual).whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        driverController.rightBumper().and(() -> isManual).whileTrue(new InstantCommand(() -> processorSubsystem.rollFlywheel(-0.6)));
        driverController.rightBumper().and(() -> isManual).whileFalse(new InstantCommand(() -> processorSubsystem.stopFlywheel()));

        new Trigger(() -> processorSubsystem.isRollerCooking()).onTrue(LEDCommands.intakenAlgea(ledSubsystem));
        new Trigger(() -> dealgeafierSubsystem.getLimitSwitch()).onTrue(LEDCommands.intakenAlgea(ledSubsystem));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}
