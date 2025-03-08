package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.constants.Constants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax coralSpinnerMotor;



    public CoralSubsystem() {
        coralSpinnerMotor = new SparkMax(Constants.CoralPivotConstants.CAN_ID, MotorType.kBrushless);
    }

    public void Coralspin(double speed) {
        coralSpinnerMotor.set(speed);
    }

    public void CoralSpinstop() {
        coralSpinnerMotor.set(0);
    }

}

