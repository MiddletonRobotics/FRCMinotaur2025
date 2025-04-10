package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.DealgeafierSubsystem.PivotingState;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;
import frc.robot.utilities.constants.Constants;

public class DealgeafierCommands {
    public static Command intakeUntilBroken(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Intake"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(-1.0), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.getLimitSwitch()),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Intaking");
    } 
    
    public static Command shootAlgea(DealgeafierSubsystem dealgeaElevatorSubsystem, LEDSubsystem ledSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Shooting"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(1), dealgeaElevatorSubsystem).until(() -> !dealgeaElevatorSubsystem.getLimitSwitch()),
            new WaitCommand(0.1),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Shooting");
    }

    public static Command shootAlgeaSensorless(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Shooting"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(1), dealgeaElevatorSubsystem).raceWith(new WaitCommand(0.5)),
            new WaitCommand(0.1),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Shooting");
    }

    public static Command runPivotToGround(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + PivotingState.GROUND),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(PivotingState.GROUND)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atTargetPosition())
        );
    }

    public static Command runPivotToReef(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + PivotingState.REEF),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(PivotingState.REEF)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atTargetPosition())
        );
    }

    public static Command runPivotToBarge(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + PivotingState.BARGE),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(PivotingState.BARGE)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atTargetPosition())
        );
    }

    public static Command runPivotToStart(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + PivotingState.START),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(PivotingState.START)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atTargetPosition())
        );
    }

    public static Command runPivotToStored(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + PivotingState.STORED),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(PivotingState.STORED)).alongWith(new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atTargetPosition())
        );
    }
}
