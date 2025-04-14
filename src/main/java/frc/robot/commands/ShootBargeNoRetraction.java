package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ShootBargeNoRetraction extends SequentialCommandGroup {
    public ShootBargeNoRetraction(ElevatorSubsystem elevatorSubsystem, DealgeafierSubsystem dealgeafierSubsystem, LEDSubsystem ledSubsystem) {
        setName("ShootBarge");
        addCommands(
            new ParallelCommandGroup(
                DealgeafierCommands.runPivotToStored(dealgeafierSubsystem),
                ElevatorCommands.runElevatorToBarge(elevatorSubsystem)
            ),
            DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem),
                    DealgeafierCommands.shootAlgea(dealgeafierSubsystem, ledSubsystem),   
            new ParallelCommandGroup(
                ElevatorCommands.runElevatorToStow(elevatorSubsystem),
                DealgeafierCommands.runPivotToStored(dealgeafierSubsystem)   
            )
        );
    }
}

