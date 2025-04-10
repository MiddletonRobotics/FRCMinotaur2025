package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    private SparkMax coralSpinnerMotor;

    private SparkMaxConfig CoralSpinnerConfiguration;

    //private RelativeEncoder algeaElevatorPivotingEncoder;
    private RelativeEncoder CoralSpinnerEncoder;

    //private SparkClosedLoopController pivotingPIDController;

    private Alert rollerCANDisconnected;
    private Alert rollerOverTempurature;
    private Alert rollerOverCurrent;
    private Alert rollerSensorDisconnected;

    private DigitalInput firstBeamBreak;
    private DigitalInput secondBeamBreak;


    public CoralSubsystem() {
        coralSpinnerMotor = new SparkMax(21, MotorType.kBrushless);

        CoralSpinnerEncoder = coralSpinnerMotor.getEncoder();
        CoralSpinnerConfiguration = new SparkMaxConfig();
        configureCoralSpinner();

        firstBeamBreak = new DigitalInput(1);
        secondBeamBreak = new DigitalInput(2);

        rollerCANDisconnected = new Alert("Coral CAN Disconnect. Will not function.", AlertType.kError);
        rollerOverTempurature = new Alert("Coral Over Tempurature", AlertType.kWarning);
        rollerOverCurrent = new Alert("Coral Over Current", AlertType.kWarning);
        rollerSensorDisconnected = new Alert("Coral Sensor Disconnected. Will not function.", AlertType.kError);
    }
    
    public void configureCoralSpinner() {
        CoralSpinnerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(70);
            CoralSpinnerConfiguration.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            CoralSpinnerConfiguration.closedLoop
            .pid(0.01, 0.0, 0.0);

        coralSpinnerMotor.configure(CoralSpinnerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        CoralSpinnerEncoder.setPosition(0.0);
    }

    public void spinCoral(double speed) {
        coralSpinnerMotor.set(speed);
    }

    public void stopCoral() {
        coralSpinnerMotor.set(0);
    }

    public void slowCoral() {
        coralSpinnerMotor.set(0.1);
    }

    public boolean firstBeamBroken() {
        return !firstBeamBreak.get();
    }

    public boolean secondBeamBreak() {
        return !secondBeamBreak.get();
    }

    @Override
    public void periodic() {
        updateLogs();

        rollerCANDisconnected.set(coralSpinnerMotor.getFaults().can);
        rollerOverTempurature.set(coralSpinnerMotor.getFaults().temperature);
        rollerOverCurrent.set(coralSpinnerMotor.getWarnings().overcurrent);
        rollerSensorDisconnected.set(coralSpinnerMotor.getFaults().sensor);
    }

    public void updateLogs() {
        SmartDashboard.putBoolean("Coral First Beam Break", firstBeamBroken());
        SmartDashboard.putBoolean("Coral Second Beam Break", secondBeamBreak());
    }
}

