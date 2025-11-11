package frc.robot.commands;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;
import frc.robot.utilities.constants.Constants;

public class CoralCommands {
  public static Command funnelIntakingUntilBroken(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, LEDSubsystem ledSubsystem) {
      return new SequentialCommandGroup(
        new PrintCommand("Automatic Coral Intaking"),  
        new RunCommand(() -> coralSubsystem.spinCoral(-0.9)).until(coralSubsystem::firstBeamBroken).andThen(new InstantCommand(() -> coralSubsystem.setCurrentlyIsStaging(true))),
        new InstantCommand(() -> ledSubsystem.setPattern(new StrobeAnimation(8, 76).withColor(Constants.LEDConstants.yellow).withSlot(0))),
        new RunCommand(() -> coralSubsystem.spinCoral(-0.3)).until(coralSubsystem::secondBeamBreak).withTimeout(3),
        new RunCommand(() -> coralSubsystem.spinCoral(-0.2)).until(() -> !coralSubsystem.firstBeamBroken()).withTimeout(4),
        new RunCommand(() -> coralSubsystem.spinCoral(0.07)).until(coralSubsystem::firstBeamBroken).withTimeout(2),
        new InstantCommand(() -> coralSubsystem.stopCoral()).andThen(new InstantCommand(() -> coralSubsystem.setCurrentlyHasCoral(true)).alongWith(new InstantCommand(() -> coralSubsystem.setCurrentlyIsStaging(false)))),
        new InstantCommand(() -> ledSubsystem.setPattern(new EmptyAnimation(0))),
        new InstantCommand(() -> ledSubsystem.setPattern(new SolidColor(8, 76).withColor(Constants.LEDConstants.green)))
      );
  }

  public static Command scoreCoral(CoralSubsystem coralSubsystem, LEDSubsystem ledSubsystem) {
    return new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> ledSubsystem.setPattern(new StrobeAnimation(8, 76).withColor(Constants.LEDConstants.yellow).withSlot(0))),
        new RunCommand(() -> coralSubsystem.spinCoral(-0.4)).until(() -> !coralSubsystem.secondBeamBreak()),
        new WaitCommand(0.3),
        new InstantCommand(() -> coralSubsystem.stopCoral()),
        new InstantCommand(() -> ledSubsystem.setPattern(new EmptyAnimation(0))),
        new InstantCommand(() -> ledSubsystem.setPattern(new SolidColor(8, 76).withColor(Constants.LEDConstants.red)))
      ), 
      new PrintCommand("Currently DoesntHave Coral"),
      coralSubsystem::currentlyHasCoral
    );
  }
}