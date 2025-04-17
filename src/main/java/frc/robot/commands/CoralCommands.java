package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;
import frc.robot.utilities.constants.Constants;

public class CoralCommands {
    public static Command funnelIntakingUntilBroken(CoralSubsystem coralSubsystem, LEDSubsystem ledSubsystem) {
        return new SequentialCommandGroup(
          new PrintCommand("Automatic Coral Intaking"),  
          new InstantCommand(() -> ledSubsystem.setPattern(BlinkinPattern.STROBE_GOLD)),
          new RunCommand(() -> coralSubsystem.spinCoral(-1), coralSubsystem).until(() -> coralSubsystem.firstBeamBroken()).withTimeout(4),
          new InstantCommand(() -> ledSubsystem.setPattern(BlinkinPattern.GOLD)),
          new RunCommand(() -> coralSubsystem.spinCoral(-0.2), coralSubsystem).until(() -> coralSubsystem.secondBeamBreak()).withTimeout(0.8),
          new RunCommand(() -> coralSubsystem.spinCoral(-0.2), coralSubsystem).until(() -> !coralSubsystem.firstBeamBroken()).withTimeout(0.8),
          new InstantCommand(() -> coralSubsystem.stopCoral()),
          new InstantCommand(() -> coralSubsystem.spinCoral(0.15)),
          new WaitCommand(0.2),
          new InstantCommand(() -> coralSubsystem.stopCoral()).alongWith(new InstantCommand(() -> ledSubsystem.setPattern(Constants.DriverConstants.DEF_PATTERN)))
        );
    }

    public static Command scoreCoral(CoralSubsystem coralSubsystem, LEDSubsystem ledSubsystem) {
      return new SequentialCommandGroup(
        new PrintCommand("Automatic Coral Scoring"), 
        new InstantCommand(() -> ledSubsystem.setPattern(BlinkinPattern.STROBE_GOLD)),
        new InstantCommand(() -> coralSubsystem.spinCoral(-0.5)),
        new WaitCommand(0.4),
        new InstantCommand(() -> coralSubsystem.stopCoral()),
        new InstantCommand(() -> coralSubsystem.spinCoral(0.5)),
        new WaitCommand(0.3),
        new InstantCommand(() -> coralSubsystem.stopCoral()).alongWith(new InstantCommand(() -> ledSubsystem.setPattern(Constants.DriverConstants.DEF_PATTERN)))
      );
  }
}
