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

    private TrapezoidProfile.Constraints elevatorConstraints;
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

        leftElevatorGearbox = new SparkMax(Constants.ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        leftElevatorEncoder = leftElevatorGearbox.getEncoder();
        elevatorPIDController = leftElevatorGearbox.getClosedLoopController();
        leftElevatorConfiguration = new SparkMaxConfig();
        configureLeftGearbox();

        rightElevatorGearbox = new SparkMax(Constants.ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        rightElevatorEncoder = rightElevatorGearbox.getEncoder();
        rightElevatorConfiguration = new SparkMaxConfig();
        configureRightGearbox();

        feedforward = new ElevatorFeedforward(
            Constants.ElevatorConstants.ElevatorFeedforwardkS,
            Constants.ElevatorConstants.ElevatorFeedforwardkG,
            Constants.ElevatorConstants.ElevatorFeedforwardkV,
            Constants.ElevatorConstants.ElevatorFeedforwardkA
        );

        elevatorConstraints = new TrapezoidProfile.Constraints(
            Constants.ElevatorConstants.LimitedMaxVelocity.in(MetersPerSecond), 
            Constants.ElevatorConstants.LimitedMacAcceleration.in(MetersPerSecondPerSecond)
        );

        profile = new ProfiledPIDController(Constants.ElevatorConstants.ElevatorProfileKp, 0.0, Constants.ElevatorConstants.ElevatorProfileKd, elevatorConstraints);
    }

    private void configureLeftGearbox() {
        leftElevatorConfiguration
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
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

    public double getVelocityRadiansPerSecond() {
        return (leftElevatorEncoder.getVelocity() + rightElevatorEncoder.getVelocity()) / 2;
    }

    public void runElevatorUp(double speed) {
        leftElevatorGearbox.set(speed);
    }

    public void runElevatorDown(double speed) {
        leftElevatorGearbox.set(speed);
    }

    public void runIntegratedController() {
        switch (state) {
            case L1:
                elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.l1Height, ControlType.kPosition);
            case L2:
                elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.l2Height, ControlType.kPosition);
            case L3:
                elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.l3Height, ControlType.kPosition);
            case L4:
                elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.l4Height, ControlType.kPosition);
            case BARGE:
                elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.barge, ControlType.kPosition);
            case DEALGEAFIER_L2:
                elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.l2Dealgeafier, ControlType.kPosition);
            case DEALGEAFIER_L3:
            elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.l3Dealgeafier, ControlType.kPosition);
            case STOW:
                elevatorPIDController.setReference(Constants.ElevatorConstants.ElevatorHeights.stow, ControlType.kPosition);
            default:
                elevatorPIDController.setReference(0, ControlType.kPosition);
        }
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
            case BARGE:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.barge);
            case DEALGEAFIER_L2:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.l2Dealgeafier);
            case DEALGEAFIER_L3:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.l3Dealgeafier);
            case STOW:
                return UtilityFunctions.withinMargin(0.01, getPositionMeters(), Constants.ElevatorConstants.ElevatorHeights.stow);
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
            case BARGE:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.barge);
                break;
            case DEALGEAFIER_L2:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.l2Dealgeafier);
                break;
            case DEALGEAFIER_L3:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.l3Dealgeafier);
                break;
            case STOW:
                setGoal(Constants.ElevatorConstants.ElevatorHeights.stow);
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
        SmartDashboard.putString("Elevator State: ", getState().toString());
        SmartDashboard.putNumber("Elevator Encoder Pos", leftElevatorEncoder.getPosition());
    }

    public void stop() {
        leftElevatorGearbox.setVoltage(0.0);
        rightElevatorGearbox.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        runState();
        logData();

        SmartDashboard.putNumber("Elevator Encoder", getPositionMeters());
    }
}