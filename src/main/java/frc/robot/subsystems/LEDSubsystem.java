package frc.robot.subsystems;

import frc.robot.utilities.BlinkinLEDController;
import frc.robot.utilities.BlinkinLEDController.BlinkinPattern;

public class LEDSubsystem {
    private BlinkinLEDController ledController;

    public LEDSubsystem() {
        System.out.println("[Initialization] Creating LEDSubsystem");

        ledController = BlinkinLEDController.getInstance();
        setAllianceColor();
    }

    public void setAllianceColor() {
        ledController.setAllianceColorBreath();
    }

    public void setPattern(BlinkinPattern pattern) {
        ledController.setPattern(pattern);
    }
}