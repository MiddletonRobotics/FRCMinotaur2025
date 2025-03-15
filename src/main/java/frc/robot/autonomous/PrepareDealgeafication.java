package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DealgeafierCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class PrepareDealgeafication extends SequentialCommandGroup {
    public PrepareDealgeafication(DealgeafierSubsystem dealgeafierSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
            ElevatorCommands.runElevatorToPosition(elevatorSubsystem, ElevatorStates.DEALGEAFIER_L2),
            DealgeafierCommands.runPivotToReef(dealgeafierSubsystem).alongWith(new InstantCommand(() -> dealgeafierSubsystem.startRolling(-1)))
        );
    }
}
