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

    public static boolean a(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.XboxController.Button.kA.value);
    }

    public static boolean b(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.XboxController.Button.kB.value);
    }

    public static boolean x(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.XboxController.Button.kX.value);
    }

    public static boolean y(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.XboxController.Button.kY.value);
    }

    public static boolean leftBumper(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.XboxController.Button.kLeftBumper.value);
    }

    public static boolean rightBumper(Joystick Controller) {
        return Controller.getRawButton(Constants.ControllerRawButtons.XboxController.Button.kRightBumper.value);
    }

    public static double leftY(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.XboxController.Axis.kLeftY.value);
    }

    public static double leftX(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.XboxController.Axis.kLeftX.value);
    }

    public static double rightY(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.XboxController.Axis.kRightY.value);
    }

    public static double rightX(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.XboxController.Axis.kRightX.value);
    }

    public static double rightTrigger(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.XboxController.Axis.kRightTrigger.value);
    }

    public static double leftTrigger(Joystick Controller) {
        return Controller.getRawAxis(Constants.ControllerRawButtons.XboxController.Axis.kLeftTrigger.value);
    }
}
