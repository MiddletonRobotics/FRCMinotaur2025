package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ClimbSubsystem;


public class ClimbCommands {
    


    public static Command moveClimbUpCommand(ClimbSubsystem climbSubsystem, DoubleSupplier speed) {
      return new SequentialCommandGroup(
        new InstantCommand(() -> climbSubsystem.ClimbMoveUp(-0.5))     
      );
    }



  public static Command moveClimbDownCommand(ClimbSubsystem climbSubsystem, DoubleSupplier speed) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> climbSubsystem.ClimbMoveDown(-0.5))     
    );
}
}
