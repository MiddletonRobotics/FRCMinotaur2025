package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.constants.Constants;


public class AlgeaGroundSubsystem extends SubsystemBase {
    private final SparkMax algeaSpinningMotor;
    private final SparkMax algeaPivotingMotor;

    public AlgeaGroundSubsystem() {
        algeaPivotingMotor = new SparkMax(Constants.AlgeaProcessorConstants.pivotingID, MotorType.kBrushless);
        algeaSpinningMotor = new SparkMax(Constants.AlgeaProcessorConstants.rollerID, MotorType.kBrushless);
    }

    public void spinPivotMotor(double speed) {
        algeaSpinningMotor.set(speed);
    }

    public void stopPivotMotor() {
        algeaSpinningMotor.set(0.0);
    }
    
}

