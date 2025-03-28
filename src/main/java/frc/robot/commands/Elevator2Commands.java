package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem2;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class Elevator2Commands {
    public static Command runElevatorToPosition(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToReset(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.setPositionGoal(ElevatorStates.STOW.getPosition())).raceWith(new WaitCommand(0.3)),
            new WaitUntilCommand(() -> elevatorSubsystem.isElevatorCooking()),
            new InstantCommand(() -> elevatorSubsystem.setPosition(0))
        );
    }

    public static Command runElevatorToStow(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.runToStow()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToL1(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L1)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToL2(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L2)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToL3(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L3)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToL4(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L4)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToBarge(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.BARGE)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToD2(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L2)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }

    public static Command runElevatorToD3(ElevatorSubsystem2 elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.DEALGEAFIER_L3)),
            new RunCommand(() -> elevatorSubsystem.runToPosition()).until(() -> elevatorSubsystem.atGoal())
        );
    }
}
