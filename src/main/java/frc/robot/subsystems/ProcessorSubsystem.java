package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.utilities.constants.Constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProcessorSubsystem extends SubsystemBase {
    private SparkMax pivotingMotor;
    private SparkMax rollerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig rollerConfiguration;

    private RelativeEncoder pivotingEncoder;
    private RelativeEncoder rollingEncoder;
    private SparkClosedLoopController pivotPIDController;

    private Alert pivotCANDisconnected;
    private Alert pivotOverTempurature;
    private Alert pivotOverCurrent;
    private Alert pivotSensorDisconnected;

    private Alert rollerCANDisconnected;
    private Alert rollerOverTempurature;
    private Alert rollerOverCurrent;
    private Alert rollerSensorDisconnected;

    private ArmFeedforward feedforward = new ArmFeedforward(
        Constants.ProcessorConstants.kS.in(Volts),
        Constants.ProcessorConstants.kG.in(Volts),
        Constants.ProcessorConstants.kV
    );   

    public enum GroundPivotingState {
        STORED(Degrees.of(91.6)),
        INTAKEN(Radians.of(2.104873)),
        GROUND(Degrees.of(151.0));

        private final Angle position;

        private GroundPivotingState(Angle position) {
            this.position = position;
        }

        public Angle getPosition() {
            return this.position;
        }
    }

    private GroundPivotingState groundPivotingState = GroundPivotingState.STORED;

    public ProcessorSubsystem() {
        pivotingMotor = new SparkMax(Constants.ProcessorConstants.pivotMotorID, MotorType.kBrushless);
        pivotingEncoder = pivotingMotor.getEncoder();
        pivotPIDController = pivotingMotor.getClosedLoopController();
        pivotingConfiguration = new SparkMaxConfig();
        configurePivotingMotor();

        rollerMotor = new SparkMax(Constants.ProcessorConstants.rollerMotorID, MotorType.kBrushless);
        rollingEncoder = rollerMotor.getEncoder();
        rollerConfiguration = new SparkMaxConfig();
        configureRollerMotor();

        pivotCANDisconnected = new Alert("Processor Pivot CAN Disconnect. Will not Function", AlertType.kError);
        pivotOverTempurature = new Alert("Processor Pivot Over Tempurature", AlertType.kWarning);
        pivotOverCurrent = new Alert("Processor Pivot Over Current", AlertType.kWarning);
        pivotSensorDisconnected = new Alert("Processor Pivot Sensor Disconnect. Will not function", AlertType.kError);

        rollerCANDisconnected = new Alert("Processor Roller CAN Disconnect. Will not Function", AlertType.kError);
        rollerOverTempurature = new Alert("Processor Roller Over Tempurature", AlertType.kWarning);
        rollerOverCurrent = new Alert("Processor Roller Over Current", AlertType.kWarning);
        rollerSensorDisconnected = new Alert("Processor Roller Sensor Disconnect. Will not function", AlertType.kError);
    }

    public void configurePivotingMotor() {
        pivotingConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(35);
        pivotingConfiguration.encoder
            .positionConversionFactor(Constants.ProcessorConstants.PositionConversionFactor)
            .velocityConversionFactor(Constants.ProcessorConstants.VelocityConversionFactor);
        pivotingConfiguration.closedLoop
            .pid(0.5, 0.0, 0.04, ClosedLoopSlot.kSlot0);

        pivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotingEncoder.setPosition(GroundPivotingState.STORED.getPosition().in(Radians));
    }

    public void configureRollerMotor() {
        rollerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(50);
        rollerConfiguration.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        rollerConfiguration.closedLoop
            .pid(0.01, 0.0, 0.0);

        rollingEncoder.setPosition(0.0);
    
    }

    @Override
    public void periodic() {
        updateLogs();

        pivotCANDisconnected.set(pivotingMotor.getFaults().can);
        pivotOverTempurature.set(pivotingMotor.getFaults().temperature);
        pivotOverCurrent.set(pivotingMotor.getWarnings().overcurrent);
        pivotSensorDisconnected.set(pivotingMotor.getFaults().sensor);

        rollerCANDisconnected.set(rollerMotor.getFaults().can);
        rollerOverTempurature.set(rollerMotor.getFaults().temperature);
        rollerOverCurrent.set(rollerMotor.getWarnings().overcurrent);
        rollerSensorDisconnected.set(rollerMotor.getFaults().sensor);
    }

    public void updateLogs() {
        SmartDashboard.putNumber("Processor Pivot Position", pivotingEncoder.getPosition());
        SmartDashboard.putNumber("Processor Pivot Motor Temp.", pivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Processor Pivot Target", groundPivotingState.getPosition().in(Radians));
        SmartDashboard.putNumber("Processor Roller Velocity", pivotingEncoder.getVelocity());
        SmartDashboard.putNumber("Processor Roller Temp.", pivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Processor Roller Stall", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Processor Roller Cooking", isRollerCooking());
        SmartDashboard.putNumber("Processor Pivot Error", calculateError());
        SmartDashboard.putBoolean("Processor At Goal", atGoal());
    }

    public void setNeutralModes(IdleMode idleMode) {
        rollerConfiguration.idleMode(idleMode);
        rollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setGoal(GroundPivotingState groundPivotingState) {
        setPivotingState(groundPivotingState);
    }

    public void runToPosition() {
        Angle targetPosition = getGroundPivotingState().getPosition();
        pivotPIDController.setReference(targetPosition.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(pivotingEncoder.getPosition(), 0.15), ArbFFUnits.kVoltage);
    }

    public boolean atGoal() {
        Angle targetPosition = getGroundPivotingState().getPosition();
        return (targetPosition.in(Radians) - pivotingEncoder.getPosition() < 0.05 && targetPosition.in(Radians) - pivotingEncoder.getPosition() > -0.05);
    }

    public double calculateError() {
        Angle targetPosition = getGroundPivotingState().getPosition();
        return targetPosition.in(Radians) - pivotingEncoder.getPosition();
    }

    public void rollFlywheel(double speed) {
        rollerMotor.set(speed);
    }

    public void stopFlywheel() {
        rollerMotor.set(-0.1);
    }

    public void startGroundPivot(double speed) {
        pivotingMotor.set(speed);
    }

    public void stopGroundPivot() {
        pivotingMotor.set(0.0);
    }

    private void setPivotingState(GroundPivotingState groundPivotingState) {
        this.groundPivotingState = groundPivotingState;
    }

    public GroundPivotingState getGroundPivotingState() {
        return groundPivotingState;
    }

    public boolean isRollerCooking() {
        double stall = rollerMotor.getOutputCurrent();

        if(stall > 9) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isBrownedOut() {
        return pivotingMotor.getStickyWarnings().brownout || rollerMotor.getStickyWarnings().brownout;
    }
}