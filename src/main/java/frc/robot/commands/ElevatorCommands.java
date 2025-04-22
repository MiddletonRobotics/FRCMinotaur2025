package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorCommands {
    public static Command runElevatorToPosition(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToReset(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.runToStow()).until(() -> elevatorSubsystem.atGoal()),
            new WaitUntilCommand(() -> elevatorSubsystem.isElevatorCooking()),
            new InstantCommand(() -> elevatorSubsystem.setPosition(0))
        );
    }

    public static Command runElevatorToStow(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.runToStow()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command stageElevatorToL1(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.runToL1()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToL1(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L1)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToL2(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L2)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToL3(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L3)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToL4(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L4)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToBarge(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.BARGE)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToD2(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L2)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }

    public static Command runElevatorToD3(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L3)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal()).withTimeout(5)
        );
    }
}
