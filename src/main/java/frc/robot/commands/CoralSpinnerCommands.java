package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralSubsystem;

public class CoralSpinnerCommands {
    
    public static Command funnelIntakingUntilBroken(CoralSubsystem coralSubsystem, DoubleSupplier speed) {
        return new SequentialCommandGroup(
          new PrintCommand("Automatic Coral Intaking")  
        );
    }
}
