package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.utilities.ShuffleData;
import frc.robot.utilities.UtilityFunctions;



//import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
//import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
//import frc.robot.subsystems.elevator.real.ElevatorSparkMax;
//import frc.robot.subsystems.elevator.sim.ElevatorSimulation;
//import frc.robot.utils.ShuffleData;
//import frc.robot.utils.UtilityFunctions;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorStates state = ElevatorStates.STOP;

    private TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.LimitedMaxVelocity, Constants.ElevatorConstants.LimitedMacAcceleration);
    private ProfiledPIDController profile;
    private ElevatorFeedforward feedforward;

    private SparkMax rightElevatorGearbox, leftElevatorGearbox;
    private RelativeEncoder rightElevatorEncoder, leftElevatorEncoder;
    private SparkClosedLoopController elevatorPIDController;
    private SparkMaxConfig rightElevatorConfiguration, leftElevatorConfiguration;

    private Function<Double, Double> wrapping = (input) -> input;
    private BiFunction<Double, Double, Boolean> deadband = (input, setpoint) -> false;

    public ElevatorSubsystem() {
        System.out.println("[Initialization] Creating ElevatorSubsystem");

        rightElevatorGearbox = new SparkMax(Constants.ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        rightElevatorEncoder = rightElevatorGearbox.getEncoder();
        rightElevatorConfiguration = new SparkMaxConfig();
        configureRightGearbox();

        leftElevatorGearbox = new SparkMax(Constants.ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        leftElevatorEncoder = leftElevatorGearbox.getEncoder();
        elevatorPIDController = leftElevatorGearbox.getClosedLoopController();
        leftElevatorConfiguration = new SparkMaxConfig();
        configureLeftGearbox();

        feedforward = new ElevatorFeedforward(
            Constants.ElevatorConstants.ElevatorFeedforwardkS,
            Constants.ElevatorConstants.ElevatorFeedforwardkG,
            Constants.ElevatorConstants.ElevatorFeedforwardkV,
            Constants.ElevatorConstants.ElevatorFeedforwardkA
        );

        profile = new ProfiledPIDController(Constants.ElevatorConstants.ElevatorProfileKp, 0.0, Constants.ElevatorConstants.ElevatorProfileKd, elevatorConstraints);
    }

    public void configureLeftGearbox() {
        leftElevatorConfiguration
            .inverted(Constants.ElevatorConstants.leftElevatorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ElevatorConstants.ElevatorContinousCurrent)
            .voltageCompensation(Constants.ElevatorConstants.ElevatorVoltageCompensation);
        leftElevatorConfiguration.encoder
            .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
            .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
        leftElevatorConfiguration.closedLoop
            .pid(Constants.ElevatorConstants.ElevatorSparkKp, 0.0, Constants.ElevatorConstants.ElevatorSparkKd);

        leftElevatorGearbox.configure(leftElevatorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void configureRightGearbox() {
        rightElevatorConfiguration
            .inverted(Constants.ElevatorConstants.rightElevatorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ElevatorConstants.ElevatorContinousCurrent)
            .voltageCompensation(Constants.ElevatorConstants.ElevatorVoltageCompensation)
            .follow(leftElevatorGearbox);
        rightElevatorConfiguration.encoder
            .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
            .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);

        rightElevatorGearbox.configure(rightElevatorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public ElevatorStates getState() {
        return state;
    }

    public double getPositionMeters() {
        double leftPosition = leftElevatorEncoder.getPosition();
        double rightPosition = rightElevatorEncoder.getPosition();

        leftPosition = wrapping.apply(leftPosition);
        rightPosition = wrapping.apply(rightPosition);

        return (leftPosition + rightPosition) / 2;
    }

    public double getVelocityRadPerSec() {
        return (leftElevatorEncoder.getVelocity() + rightElevatorEncoder.getVelocity()) / 2;
    }

    /** returns true when the state is reached */
    public boolean atSetpointState() {
        switch (state) {
            case L1:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.l1Height);
            case L2:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.l2Height);
            case L3:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.l3Height);
            case L4:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.l4Height);
            case MAX:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.MaximumHeightMeters);
            case STOW:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.BaseHeight);
            default:
                return false;
        }
    }

    public void setState(ElevatorStates state) {
        this.state = state;
        switch (state) {
            case STOP:
                stop();
                break;
            case L1:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.l1Height);
                break;
            case L2:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.l2Height);
                break;
            case L3:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.l3Height);
                break;
            case L4:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.l4Height);
                break;
            case MAX:
                setGoal(Constants.ElevatorConstants.MaximumHeightMeters);
                break;
            case STOW:
                setGoal(Constants.ElevatorConstants.BaseHeight);
                break;
            default:
                setGoal(0);
                break;
        }
    }

    public void setGoal(double height) {
        profile.setGoal(height);
    }

    public void setVoltage(double volts) {
        double inputVolts = MathUtil.applyDeadband(volts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12.0, 12.0);
        leftElevatorGearbox.setVoltage(inputVolts);
        rightElevatorGearbox.setVoltage(inputVolts);
    }

    private void runState() {
        switch (state) {
            case STOP:
                stop();
                break;
            default:
                moveToGoal();
                break;
        }
    }

    private void moveToGoal() {
        State firstState = profile.getSetpoint();
        profile.calculate(getPositionMeters());

        State nextState = profile.getSetpoint();
        double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

        elevatorPIDController.setReference(firstState.position, ControlType.kPosition, ClosedLoopSlot.kSlot3, ffVoltage);
    }

     private void logData() {
        Logger.recordOutput("Elevator/Current Command", this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        Logger.recordOutput("Elevator/Position", getPositionMeters());
        Logger.recordOutput("Elevator/Velocity", getVelocityRadPerSec());
        //Logger.recordOutput("subsystems/elevator/acceleration", data.accelerationMetersPerSecondSquared);
        //Logger.recordOutput("subsystems/elevator/input volts", ((data.leftAppliedVolts + data.rightAppliedVolts) / 2.0));
        Logger.recordOutput("Elevator/Left Applied Output", leftElevatorGearbox.getAppliedOutput());
        Logger.recordOutput("Elevator/Left Applied Output", rightElevatorGearbox.getAppliedOutput());
        Logger.recordOutput("Elevator/Left Output Current", leftElevatorGearbox.getOutputCurrent());
        Logger.recordOutput("Elevator/Right Output Current", rightElevatorGearbox.getOutputCurrent());
        Logger.recordOutput("Elevator/Left Motor Tempurature", leftElevatorGearbox.getMotorTemperature());
        Logger.recordOutput("Elevator/Right Motor Tempurature", rightElevatorGearbox.getMotorTemperature());
    }

    public void stop() {
        leftElevatorGearbox.setVoltage(0.0);
        rightElevatorGearbox.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        runState();
        logData();
    }
}