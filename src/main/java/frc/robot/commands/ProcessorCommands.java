package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.subsystems.ProcessorSubsystem;
import frc.robot.subsystems.ProcessorSubsystem.GroundPivotingState;

public class ProcessorCommands {
    public static Command startRollCommand(ProcessorSubsystem algeaGroundSubsystem, DoubleSupplier speed) {
        return algeaGroundSubsystem.run(
            () -> algeaGroundSubsystem.rollFlywheel(speed.getAsDouble())
        );
    }

    public static Command startRollCommand(ProcessorSubsystem algeaGroundSubsystem, double speed) {
        return startRollCommand(algeaGroundSubsystem, () -> speed);

    }

    //Manual Pivot
    public static Command startPivotToGround(ProcessorSubsystem algeaGroundSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Intake Algea Command"),
            new InstantCommand(() -> algeaGroundSubsystem.setGoal(GroundPivotingState.GROUND)),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition()).until(() -> algeaGroundSubsystem.atGoal())
        );
    }

    public static Command startPivotToIntaken(ProcessorSubsystem algeaGroundSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Intake Algea Command"),
            new InstantCommand(() -> algeaGroundSubsystem.setGoal(GroundPivotingState.INTAKEN)),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition()).until(() -> algeaGroundSubsystem.atGoal())
        );
    }

    public static Command startPivotToStored(ProcessorSubsystem algeaGroundSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Intake Algea Command"),
            new InstantCommand(() -> algeaGroundSubsystem.setGoal(GroundPivotingState.STORED)),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition()).until(() -> algeaGroundSubsystem.atGoal())
        );
    }

    public static Command intakeAlgea(ProcessorSubsystem algeaGroundSubsystem, GroundPivotingState groundPivotingState) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Intake Algea Command"),
            new InstantCommand(() -> algeaGroundSubsystem.setGoal(groundPivotingState)).alongWith(new InstantCommand(() -> algeaGroundSubsystem.rollFlywheel(-1))),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition()).until(() -> algeaGroundSubsystem.atGoal()),
            new WaitUntilCommand(() -> algeaGroundSubsystem.isRollerCooking()),
            new WaitCommand(0.5),
            new InstantCommand(() -> algeaGroundSubsystem.setGoal(GroundPivotingState.STORED)).alongWith(new InstantCommand(() -> algeaGroundSubsystem.stopFlywheel())),
            new RunCommand(() -> algeaGroundSubsystem.runToPosition()).raceWith(new WaitCommand(0.4))
        );
    }

    public static Command spitOutBall(ProcessorSubsystem algeaGroundSubsystem) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Spit out Algea Command"),
            new InstantCommand(() -> algeaGroundSubsystem.rollFlywheel(1)),
            new WaitCommand(0.5),
            new InstantCommand(() -> algeaGroundSubsystem.stopFlywheel())
        );
    }
}
