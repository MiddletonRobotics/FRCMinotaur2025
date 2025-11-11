package frc.robot.commands;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;
import frc.robot.utilities.constants.Constants;

public class LEDCommands {
    /* 
    public static Command robotReadyForMatch(LEDSubsystem ledSubsystem) {
        return Commands.runOnce(() -> {
            ledSubsystem.setPattern(new LarsonAnimation(8, 76).withColor(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? RGBWColor.fromHex("#FF0000").get() : RGBWColor.fromHex("#FF0000").get()
            ));
        }, ledSubsystem);
    }
        */
}
