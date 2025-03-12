package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.DealgeafierSubsystem.PivotingState;

public class DealgeafierCommands {
    public static Command intakeUntilBroken(DealgeafierSubsystem dealgeaElevatorSubsystem, double speed) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Intake"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(-speed), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.getLimitSwitch()),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Intaking");
    } 
    
    public static Command shootAlgea(DealgeafierSubsystem dealgeaElevatorSubsystem, double speed) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Auto Algae Shooting"),
            new RunCommand(() -> dealgeaElevatorSubsystem.startRolling(speed), dealgeaElevatorSubsystem).until(() -> !dealgeaElevatorSubsystem.getLimitSwitch()),
            new WaitCommand(0.1),
            new InstantCommand(() -> dealgeaElevatorSubsystem.stopRolling())
        ).withName("Auto Algea Shooting");
    }

    public static Command runPivotToPosition(DealgeafierSubsystem dealgeaElevatorSubsystem, PivotingState pivotingState) {
        return new SequentialCommandGroup(
            new PrintCommand("Running the Algae Pivot to: " + pivotingState),
            new InstantCommand(() -> dealgeaElevatorSubsystem.setPivotingState(pivotingState)),
            new RunCommand(() -> dealgeaElevatorSubsystem.runToPosition(), dealgeaElevatorSubsystem).until(() -> dealgeaElevatorSubsystem.atSetpoint())
        );
    }
}
