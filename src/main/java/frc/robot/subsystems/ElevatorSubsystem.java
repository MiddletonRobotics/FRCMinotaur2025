package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.BiFunction;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.utilities.ShuffleData;
import frc.robot.utilities.TrapezoidController;
import frc.robot.utilities.UtilityFunctions;



//import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
//import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
//import frc.robot.subsystems.elevator.real.ElevatorSparkMax;
//import frc.robot.subsystems.elevator.sim.ElevatorSimulation;
//import frc.robot.utils.ShuffleData;
//import frc.robot.utils.UtilityFunctions;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorStates state = ElevatorStates.STOP;

    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    private PIDController pidController;
    private TrapezoidController trapezoidController;
    private ElevatorFeedforward feedforward;

    private SparkMax rightElevatorGearbox, leftElevatorGearbox;
    private RelativeEncoder rightElevatorEncoder, leftElevatorEncoder;
    private SparkClosedLoopController leftElevatorPID, rightElevatorPID;
    private SparkMaxConfig rightElevatorConfiguration, leftElevatorConfiguration;

    private Function<Double, Double> wrapping = (input) -> input;
    private BiFunction<Double, Double, Boolean> deadband = (input, setpoint) -> false;
    private ElevatorStates elevatorState;
    private double pidVal, FFVal, outputVoltage;

    public ElevatorSubsystem() {
        System.out.println("[Initialization] Creating ElevatorSubsystem");

        leftElevatorGearbox = new SparkMax(Constants.ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        leftElevatorEncoder = leftElevatorGearbox.getEncoder();
        leftElevatorPID = leftElevatorGearbox.getClosedLoopController();
        leftElevatorConfiguration = new SparkMaxConfig();
        configureLeftGearbox();

        rightElevatorGearbox = new SparkMax(Constants.ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        rightElevatorEncoder = rightElevatorGearbox.getEncoder();
        rightElevatorPID = rightElevatorGearbox.getClosedLoopController();
        rightElevatorConfiguration = new SparkMaxConfig();
        configureRightGearbox();

        feedforward = new ElevatorFeedforward(
            Constants.ElevatorConstants.ElevatorFeedforwardkS,
            Constants.ElevatorConstants.ElevatorFeedforwardkG,
            Constants.ElevatorConstants.ElevatorFeedforwardkV,
            Constants.ElevatorConstants.ElevatorFeedforwardkA
        );

        pidController = new PIDController(Constants.ElevatorConstants.ElevatorProfileKp, 0.0, Constants.ElevatorConstants.ElevatorProfileKd);
        pidController.enableContinuousInput(0, 12);
        pidController.setTolerance(0.1);

        trapezoidController = new TrapezoidController(0.0, 0.05, .1, 3.5, 3, 7.5, 0.4); 
        setElevatorState(ElevatorStates.STOW);
    }

    private void configureLeftGearbox() {
        leftElevatorConfiguration
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .voltageCompensation(Constants.ElevatorConstants.ElevatorVoltageCompensation);
        leftElevatorConfiguration.encoder
            .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
            .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
        leftElevatorConfiguration.closedLoop
            .pid(Constants.ElevatorConstants.ElevatorSparkKp, 0.0, Constants.ElevatorConstants.ElevatorSparkKd)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        leftElevatorGearbox.configure(leftElevatorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftElevatorEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        pidVal = pidController.calculate(getPositionMeters(), getElevatorState().getPosition());
        FFVal = feedforward.calculate(0.15);

        outputVoltage = pidVal + FFVal;
    }

    private void configureRightGearbox() {
        rightElevatorConfiguration
            .apply(leftElevatorConfiguration)
            .follow(16, true);

        rightElevatorGearbox.configure(rightElevatorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public ElevatorStates getState() {
        return state;
    }

    public double getPositionMeters() {
        double leftPosition = leftElevatorEncoder.getPosition();
        double rightPosition = rightElevatorEncoder.getPosition();

        return (leftPosition + rightPosition) / 2;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public boolean integratedPIDatSetpoint() {
        return Math.abs(getElevatorState().getPosition() - getPositionMeters()) < 0.2;
    }

    public double getVelocityRadiansPerSecond() {
        return (leftElevatorEncoder.getVelocity() + rightElevatorEncoder.getVelocity()) / 2;
    }

    public ElevatorStates getElevatorState() {
        return elevatorState;
    }

    public void setElevatorState(ElevatorStates elevatorState) {
        this.elevatorState = elevatorState;
    }

    public void runElevatorToPosition() {
        leftElevatorGearbox.setVoltage(outputVoltage);
    }

    public void runElevatorIntegratedPID() {
        leftElevatorPID.setReference(getElevatorState().getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
        rightElevatorPID.setReference(getElevatorState().getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void runElevatorUp(double speed) {
        leftElevatorGearbox.set(speed);
    }

    public void runElevatorDown(double speed) {
        leftElevatorGearbox.set(speed);
    }

    public void stop() {
        leftElevatorGearbox.setVoltage(0.0);
        rightElevatorGearbox.setVoltage(0.0);
    }
}