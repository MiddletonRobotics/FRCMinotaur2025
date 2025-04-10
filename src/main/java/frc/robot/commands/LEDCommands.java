package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;
import frc.robot.utilities.constants.Constants;

public class LEDCommands {
    public static Command intakenAlgea(LEDSubsystem ledSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> ledSubsystem.setPattern(BlinkinPattern.STROBE_BLUE)),
            new WaitCommand(1.25),
            new InstantCommand(() -> ledSubsystem.setPattern(Constants.DriverConstants.DEF_PATTERN))
        );
    }
}
