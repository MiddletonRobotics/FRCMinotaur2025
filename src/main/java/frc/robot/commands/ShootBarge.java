package frc.robot.commands;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ShootBarge extends SequentialCommandGroup {
    public ShootBarge(ElevatorSubsystem elevatorSubsystem, DealgeafierSubsystem dealgeafierSubsystem, LEDSubsystem ledSubsystem) {
        setName("ShootBarge");
        addCommands(
            new ParallelCommandGroup(
                DealgeafierCommands.runPivotToBarge(dealgeafierSubsystem),
                ElevatorCommands.runElevatorToBarge(elevatorSubsystem)
            ),
            DealgeafierCommands.shootAlgea(dealgeafierSubsystem, ledSubsystem),
            new ParallelCommandGroup(
                ElevatorCommands.runElevatorToStow(elevatorSubsystem),
                DealgeafierCommands.runPivotToStored(dealgeafierSubsystem)   
            )
        );
    }
}
