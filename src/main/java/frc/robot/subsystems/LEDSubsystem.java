package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.utilities.BlinkinLEDController;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;

public class LEDSubsystem {
    private CANdle candle;
    private CANdleConfiguration configuration;

    public LEDSubsystem() {
        System.out.println("[Initialization] Creating LEDSubsystem");

        candle = new CANdle(23, "*");
        configuration = new CANdleConfiguration()
            .withLED(new LEDConfigs().withStripType(StripTypeValue.GRBW)
        );

        candle.getConfigurator().apply(configuration);
    }

    public void setPattern(ControlRequest request) {
        candle.setControl(request);
    }
}