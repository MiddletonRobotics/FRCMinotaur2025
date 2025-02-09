// Import information on controls (joystick, buttons) to set up each operation.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utilities.constants.Constants;

public class Controller {
    private static final Joystick DriverController = new Joystick(Constants.DriverConstants.driverControllerPort);
    private static final Joystick OperatorController = new Joystick(Constants.DriverConstants.operatorControllerPort);

    public static Joystick getDriverController() {
        return DriverController;
    }

    public static Joystick getOperatorController() {
        return OperatorController;
    }
}
