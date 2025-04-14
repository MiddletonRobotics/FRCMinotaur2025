package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RetractElevatorSafe extends SequentialCommandGroup {
    public RetractElevatorSafe(ElevatorSubsystem elevatorSubsystem, DealgeafierSubsystem dealgeafierSubsystem, LEDSubsystem ledSubsystem) {
        setName("ShootBarge");
        addCommands(
            new ParallelCommandGroup(
                ElevatorCommands.runElevatorToStow(elevatorSubsystem),
                DealgeafierCommands.runPivotToStored(dealgeafierSubsystem)   
            )
        );
    }
}

