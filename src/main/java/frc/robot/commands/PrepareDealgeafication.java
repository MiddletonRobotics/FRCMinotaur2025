package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PrepareDealgeafication extends SequentialCommandGroup {
    public PrepareDealgeafication(DealgeafierSubsystem dealgeafierSubsystem, ElevatorSubsystem elevatorSubsystem2) {
        setName("Prepare Dealgeafication");
        addCommands(
            ElevatorCommands.runElevatorToD2(elevatorSubsystem2).alongWith(new InstantCommand(() -> dealgeafierSubsystem.startRolling(-1))).alongWith(DealgeafierCommands.runPivotToReef(dealgeafierSubsystem))
        );
    }
}
