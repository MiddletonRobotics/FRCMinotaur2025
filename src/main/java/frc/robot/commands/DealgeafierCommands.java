package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgeaElevatorSubsystem;
import frc.robot.subsystems.AlgeaElevatorSubsystem.PivotingState;

public class DealgeafierCommands {
    public static Command intakeUntilBroken(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, DoubleSupplier speed) {
        return dealgeaElevatorSubsystem.run(() -> 
            dealgeaElevatorSubsystem.startRolling(speed.getAsDouble())
        ).until(() -> dealgeaElevatorSubsystem.firstLimitBroken()).andThen(() -> 
            dealgeaElevatorSubsystem.stopRolling()
        ).withName("Intake Until Limit Triggered");
    } 
    
    public static Command intakeUntilBroken(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, double speed) {
        return intakeUntilBroken(dealgeaElevatorSubsystem, () -> speed);
    }
    
    public static Command shootAlgea(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, DoubleSupplier speed) {
        return dealgeaElevatorSubsystem.run(() -> 
            dealgeaElevatorSubsystem.startRolling(-speed.getAsDouble())
        ).until(() -> dealgeaElevatorSubsystem.firstLimitBroken()).andThen(
            new WaitCommand(1).andThen(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Shoot Algea in net");
    }

    public static Command shootAlgea(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, double speed) {
        return shootAlgea(dealgeaElevatorSubsystem, () -> speed);
    }

    public static Command runPivotToPosition(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, PivotingState pivotingState) {
        return dealgeaElevatorSubsystem.run(() -> 
            dealgeaElevatorSubsystem.setPivotingState(pivotingState)
        ).andThen(() -> dealgeaElevatorSubsystem.runToPosition());  
    }
}
