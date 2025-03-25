package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem2;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class PrepareDealgeafication extends SequentialCommandGroup {
    public PrepareDealgeafication(DealgeafierSubsystem dealgeafierSubsystem, ElevatorSubsystem2 elevatorSubsystem2) {
        setName("Prepare Dealgeafication");
        addCommands(
            Elevator2Commands.runElevatorToD2(elevatorSubsystem2),
            DealgeafierCommands.runPivotToReef(dealgeafierSubsystem).alongWith(new InstantCommand(() -> dealgeafierSubsystem.startRolling(-1)))
        );
    }
}
