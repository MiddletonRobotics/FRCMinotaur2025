package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgeaElevatorSubsystem;
import frc.robot.subsystems.AlgeaGroundSubsystem;
import frc.robot.subsystems.AlgeaElevatorSubsystem.PivotingState;
import frc.robot.subsystems.AlgeaGroundSubsystem.GroundPivotingState;
import frc.robot.subsystems.CoralSubsystem;

public class AlgeaGroundCommands {
    
    public static Command runGroundPivotToPosition(AlgeaGroundSubsystem algeaGroundSubsystem, GroundPivotingState groundPivotingState) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> algeaGroundSubsystem.setPivotingState(groundPivotingState)),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition(), algeaGroundSubsystem).until(() -> algeaGroundSubsystem.atSetpoint())
        );
    }

    public static Command startRollCommand(AlgeaGroundSubsystem algeaGroundSubsystem, DoubleSupplier speed) {
        return algeaGroundSubsystem.run(
            () -> algeaGroundSubsystem.startRolling(speed.getAsDouble())
        );
    }

    public static Command startRollCommand(AlgeaGroundSubsystem algeaGroundSubsystem, double speed) {
        return startRollCommand(algeaGroundSubsystem, () -> speed);

    }




    //Manual Pivot
    public static Command startPivotCommand(AlgeaGroundSubsystem algeaGroundSubsystem, DoubleSupplier speed) {
        return algeaGroundSubsystem.run(
            () -> algeaGroundSubsystem.startGroundPivot(speed.getAsDouble())
        );
    }

    public static Command startPivotCommand(AlgeaGroundSubsystem algeaGroundSubsystem, double speed) {
        return startRollCommand(algeaGroundSubsystem, () -> speed);

    }

}
