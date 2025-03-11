package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgeaElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgeaElevatorSubsystem.PivotingState;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorCommands {
    


    public static Command runElevatorToPosition(ElevatorSubsystem elevatorSubsystem, ElevatorStates elevatorStates) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setState(elevatorStates.L2))
        );
    }
}
