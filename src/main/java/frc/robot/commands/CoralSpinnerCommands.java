package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class CoralSpinnerCommands {
    
    public static Command CoralintakeUntilBroken(CoralSubsystem coralSubsystem, DoubleSupplier speed) {
        return coralSubsystem.run(() -> 
            coralSubsystem.Coralspin(speed.getAsDouble())
        ).until(() -> coralSubsystem.CoralfirstLimitBroken()).andThen(() -> coralSubsystem.CoralSpinstop());
    }
    
}
