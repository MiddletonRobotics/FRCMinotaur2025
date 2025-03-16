package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorCommands {
    public static Command runElevatorToPosition(ElevatorSubsystem elevatorSubsystem, ElevatorStates elevatorStates) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(elevatorStates)),
            new RunCommand(() -> elevatorSubsystem.runElevatorToPosition()).until(() -> elevatorSubsystem.atSetpoint())
        );
    }

    public static Command runElevatorwithIntegratedController(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new RunCommand(() -> elevatorSubsystem.runElevatorIntegratedPID()).until(() -> elevatorSubsystem.atSetpoint())
        );
    }

    public static Command autoPrepareL4(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L4)),
            new RunCommand(() -> elevatorSubsystem.runElevatorIntegratedPID()).until(() -> elevatorSubsystem.atSetpoint())
        );
    }

    public static Command autoPrepareL3(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L3)),
            new RunCommand(() -> elevatorSubsystem.runElevatorIntegratedPID()).until(() -> elevatorSubsystem.atSetpoint())
        );
    }

    public static Command autoPrepareL1(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.L1)),
            new RunCommand(() -> elevatorSubsystem.runElevatorIntegratedPID()).until(() -> elevatorSubsystem.atSetpoint())
        );
    }

    public static Command autoStowElevator(ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorStates.STOW)),
            new RunCommand(() -> elevatorSubsystem.runElevatorIntegratedPID()).until(() -> elevatorSubsystem.atSetpoint())
        );
    }
}
