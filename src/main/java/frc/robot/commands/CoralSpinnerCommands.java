package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralSubsystem;

public class CoralSpinnerCommands {
    public static Command funnelIntakingUntilBroken(CoralSubsystem coralSubsystem, DoubleSupplier speed) {
        return new SequentialCommandGroup(
          new PrintCommand("Automatic Coral Intaking"),  
          new RunCommand(() -> coralSubsystem.spinCoral(-0.8), coralSubsystem).until(() -> coralSubsystem.firstBeamBroken()),
          new RunCommand(() -> coralSubsystem.spinCoral(-0.6), coralSubsystem).until(() -> coralSubsystem.secondBeamBreak()),
          new InstantCommand(() -> coralSubsystem.stopCoral()),
          new WaitCommand(0.1),
          new InstantCommand(() -> coralSubsystem.spinCoral(-0.155)),
          new WaitCommand(0.25),
          new InstantCommand(() -> coralSubsystem.stopCoral())
        );
    }

    public static Command scoreCoral(CoralSubsystem coralSubsystem, DoubleSupplier speed) {
      return new SequentialCommandGroup(
        new PrintCommand("Automatic Coral Scoring"), 
        new InstantCommand(() -> coralSubsystem.spinCoral(-0.5)),
        new WaitCommand(0.4),
        new InstantCommand(() -> coralSubsystem.stopCoral())
      );
  }
}
