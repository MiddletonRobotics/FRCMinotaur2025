package frc.robot.commands;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgeaElevatorSubsystem;
import frc.robot.subsystems.AlgeaElevatorSubsystem.PivotingState;

public class DealgeafierCommands {
    public static Command intakeUntilBroken(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, double speed) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Intake"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(-speed), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.firstLimitBroken()),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Intaking");
    } 
    
    public static Command shootAlgea(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, double speed) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Shooting"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(speed), dealgeaElevatorSubsystem).until(() -> !dealgeaElevatorSubsystem.firstLimitBroken()),
            new WaitCommand(0.1),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Shooting");
    }

    public static Command runPivotToPosition(AlgeaElevatorSubsystem dealgeaElevatorSubsystem, PivotingState pivotingState) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + pivotingState),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(pivotingState)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atSetpoint())
        );
    }
}
