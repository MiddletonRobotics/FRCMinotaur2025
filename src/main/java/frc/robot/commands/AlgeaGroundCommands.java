package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.subsystems.AlgeaGroundSubsystem;
import frc.robot.subsystems.AlgeaGroundSubsystem.GroundPivotingState;

public class AlgeaGroundCommands {
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

    public static Command runGroundPivotToPosition(AlgeaGroundSubsystem algeaGroundSubsystem, GroundPivotingState groundPivotingState) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + groundPivotingState),
            new InstantCommand(() -> algeaGroundSubsystem.setPivotingState(groundPivotingState)).alongWith(new InstantCommand(() -> algeaGroundSubsystem.startRolling(-1))),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition(), algeaGroundSubsystem).raceWith(new WaitCommand(0.8)),
            new WaitUntilCommand(() -> algeaGroundSubsystem.isRollerCooking()),
            new PrintCommand("Ball Intaken"),
            new InstantCommand(() -> algeaGroundSubsystem.setPivotingState(GroundPivotingState.GROUND)).andThen(new InstantCommand(() -> algeaGroundSubsystem.stopRolling())),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition(), algeaGroundSubsystem)
        );
    }
}
