package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.DealgeafierSubsystem.PivotingState;

public class DealgeafierCommands {
    public static Command intakeUntilBroken(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Intake"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(-1), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.getLimitSwitch()),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Intaking");
    } 
    
    public static Command shootAlgea(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Shooting"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(1), dealgeaElevatorSubsystem).until(() -> !dealgeaElevatorSubsystem.getLimitSwitch()),
            new WaitCommand(0.1),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Shooting");
    }

    public static Command runPivotToReef(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + PivotingState.REEF),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(PivotingState.REEF)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atGoal())
        );
    }

    public static Command runPivotToStored(DealgeafierSubsystem dealgeaElevatorSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + PivotingState.STORED),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(PivotingState.STORED)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atGoal())
        );
    }
}
