package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.constants.Constants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax coralMotor;

    public CoralSubsystem() {
        coralMotor = new SparkMax(Constants.CoralConstants.CAN_ID, MotorType.kBrushless);
    }

    public void spin(double speed) {
        coralMotor.set(speed);
    }

    public void stop() {
        coralMotor.set(0);
    }
}