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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.utilities.ShuffleData;
import frc.robot.utilities.UtilityFunctions;



public class AlgeaElevatorSubsystem extends SubsystemBase {
    private SparkMax algeaElevatorPivotingMotor;
    private SparkMax algeaElevatorRollerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig rollerConfiguration;

    private RelativeEncoder algeaElevatorPivotingEncoder;
    private RelativeEncoder algeaElevatorRollingEncoder;

    private SparkClosedLoopController pivotingPIDController;
    private SparkClosedLoopController rollerPIDController;

    private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

    private DigitalInput algeaLimitSwitch;

    private ShuffleboardTab logger; 

    private enum PivotingState {
        STORED(0),
        BARGE(0),
        REEF(0),
        GROUND(0);

        private final double position;

        private PivotingState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public PivotingState pivotingState = PivotingState.STORED;

    public AlgeaElevatorSubsystem() {
        algeaElevatorPivotingMotor = new SparkMax(19, MotorType.kBrushless);
        algeaElevatorPivotingEncoder = algeaElevatorPivotingMotor.getEncoder();
        pivotingPIDController = algeaElevatorPivotingMotor.getClosedLoopController();
        configurePivotingMotor();

        algeaElevatorRollerMotor = new SparkMax(21, MotorType.kBrushless);
        algeaElevatorRollingEncoder = algeaElevatorRollerMotor.getEncoder();
        rollerPIDController = algeaElevatorRollerMotor.getClosedLoopController();
        configureRollerMotor();

        //algeaLimitSwitch = new DigitalInput(0);
        logger = Shuffleboard.getTab(getName());
    }

    public void configurePivotingMotor() {
        pivotingConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(20);
        pivotingConfiguration.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        pivotingConfiguration.closedLoop
            .pid(0.01, 0.0, 0.0);

        algeaElevatorPivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algeaElevatorPivotingEncoder.setPosition(0.0);
    }

    public void configureRollerMotor() {
        rollerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(20);
        rollerConfiguration.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        rollerConfiguration.closedLoop
            .pid(0.01, 0.0, 0.0);

        algeaElevatorRollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algeaElevatorRollingEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        logger.add("Pivoting Motor Encoder Position", algeaElevatorPivotingEncoder.getPosition());
        logger.add("Pivoting Motor Tempurature", algeaElevatorPivotingMotor.getMotorTemperature());

        logger.add("Roller Motor Velocity", algeaElevatorPivotingEncoder.getVelocity());
        logger.add("Roller Motor RPM", algeaElevatorPivotingMotor.get());
        logger.add("Roller Motor Tempurature", algeaElevatorPivotingMotor.getMotorTemperature());
    }

    public void startRolling(double speed) {
        algeaElevatorRollerMotor.set(speed);
    }

    public void stopRolling() {
        algeaElevatorRollerMotor.set(0.0);
    }

    public void runToPosition() {
        double targetPosition = pivotingState.getPosition();
        pivotingPIDController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
