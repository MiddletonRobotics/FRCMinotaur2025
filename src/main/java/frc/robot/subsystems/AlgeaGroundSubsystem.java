package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.constants.Constants;


public class AlgeaGroundSubsystem extends SubsystemBase {
    private final SparkMax AlgeaSpinnerMotor;
    private final SparkMax AlgeaPivot1Motor;
    //private final SparkMax AlgeaPivot2Motor;




    public AlgeaGroundSubsystem() {
        AlgeaPivot1Motor = new SparkMax(Constants.AlgeaPivot1Constants.CAN_ID, MotorType.kBrushless);
        //AlgeaPivot2Motor = new SparkMax(Constants.AlgeaPivot2Constants.CAN_ID, MotorType.kBrushless);

        AlgeaSpinnerMotor = new SparkMax(Constants.AlgeaSpinnerConstants.CAN_ID, MotorType.kBrushless);
    }

    public void Algeaspin(double speed) {
        AlgeaSpinnerMotor.set(speed);



    }

    public void AlgeaSpinstop() {
        AlgeaSpinnerMotor.set(0);
    }






    public void AlgeaPivot(double speed) {
        AlgeaPivot1Motor.set(speed);
        //AlgeaPivot2Motor.set(speed);




    }

    public void AlgeaPivotstop(double speed) {
        AlgeaPivot1Motor.set(0);
        //AlgeaPivot2Motor.set(0);

    }
    
}

