package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
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

    private CANrange canrange;
    private DigitalInput secondBeamBreak;

    private CANrangeConfiguration configuration;

    private final StatusSignal<Double> beamBreakSignalStrength;
    private final StatusSignal<Boolean> beamBreakTripped;
    private final StatusSignal<Distance> beamBreakDistance;

    private boolean currentlyHasCoral = false;
    private boolean currentlyIsStaging = false;

    public CoralSubsystem() {
        coralSpinnerMotor = new SparkMax(21, MotorType.kBrushless);

        CoralSpinnerEncoder = coralSpinnerMotor.getEncoder();
        CoralSpinnerConfiguration = new SparkMaxConfig();
        configureCoralSpinner();

        configuration = new CANrangeConfiguration()
        .withProximityParams(new ProximityParamsConfigs()
            .withProximityHysteresis(0.001)
            .withProximityThreshold(0.11).withMinSignalStrengthForValidMeasurement(5500)
        );

        canrange = new CANrange(24);
        secondBeamBreak = new DigitalInput(2);

        canrange.getConfigurator().apply(configuration);

        beamBreakSignalStrength = canrange.getSignalStrength();
        beamBreakTripped = canrange.getIsDetected();
        beamBreakDistance = canrange.getDistance();

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
        return beamBreakTripped.getValue();
    }

    public boolean secondBeamBreak() {
        return !secondBeamBreak.get();
    }

    public boolean currentlyHasCoral() {
        return currentlyHasCoral;
    }

    public boolean currentlyIsStaging(){
        return currentlyIsStaging;
    }

    public void setCurrentlyHasCoral(boolean holding) {
        currentlyHasCoral = holding;
    }

    public void setCurrentlyIsStaging(boolean staging) {
        currentlyIsStaging = staging;
    }
    

    @Override
    public void periodic() {
        updateLogs();
        updateAlerts();

        BaseStatusSignal.refreshAll(beamBreakDistance, beamBreakSignalStrength, beamBreakTripped);
    }

    public void updateLogs() {
        SmartDashboard.putBoolean("Coral First Beam Broken", firstBeamBroken());
        SmartDashboard.putNumber("Coral First Beam Distance", beamBreakDistance.getValue().in(Meters));
        SmartDashboard.putBoolean("Coral Second Beam Broken", secondBeamBreak());
    }

    public void updateAlerts() {
        rollerCANDisconnected.set(coralSpinnerMotor.getFaults().can);
        rollerOverTempurature.set(coralSpinnerMotor.getFaults().temperature);
        rollerOverCurrent.set(coralSpinnerMotor.getWarnings().overcurrent);
        rollerSensorDisconnected.set(coralSpinnerMotor.getFaults().sensor);
    }
}

