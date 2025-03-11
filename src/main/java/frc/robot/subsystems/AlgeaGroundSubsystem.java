package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.AlgeaElevatorSubsystem.PivotingState;
import frc.robot.utilities.constants.Constants;



import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgeaGroundSubsystem extends SubsystemBase {
    private SparkMax algeaGroundPivotingMotor;
    private SparkMax algeaGroundRollerMotor;

    private SparkMaxConfig groundPivotingConfiguration;
    private SparkMaxConfig groundRollerConfiguration;

    private RelativeEncoder algeaGroundPivotingEncoder;
    private RelativeEncoder algeaGroundRollingEncoder;

    private SparkClosedLoopController groundPivotingPIDController;
    private SparkClosedLoopController groundRollerPIDController;


    private ShuffleboardTab logger;

    public enum GroundPivotingState {
        STORED(0),
        BARGE(0),
        REEF(31.619),
        GROUND(0);

        private final double position;

        private GroundPivotingState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    private GroundPivotingState groundPivotingState = GroundPivotingState.STORED;

    public AlgeaGroundSubsystem() {
        algeaGroundPivotingMotor = new SparkMax(17, MotorType.kBrushless);
        algeaGroundPivotingEncoder = algeaGroundPivotingMotor.getEncoder();
        groundPivotingPIDController = algeaGroundPivotingMotor.getClosedLoopController();
        groundPivotingConfiguration = new SparkMaxConfig();
        configurePivotingMotor();

        algeaGroundRollerMotor = new SparkMax(18, MotorType.kBrushless);
        algeaGroundRollingEncoder = algeaGroundRollerMotor.getEncoder();
        groundRollerPIDController = algeaGroundRollerMotor.getClosedLoopController();
        groundRollerConfiguration = new SparkMaxConfig();
        configureRollerMotor();

        logger = Shuffleboard.getTab(getName());
    }

    public void configurePivotingMotor() {
        groundPivotingConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(35);
            groundPivotingConfiguration.encoder
            .positionConversionFactor(42/18)
            .velocityConversionFactor(1);
            groundPivotingConfiguration.closedLoop
            .pid(0.03, 0.0, 0.0);

        algeaGroundPivotingMotor.configure(groundPivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algeaGroundPivotingEncoder.setPosition(0.0);
    }

    public void configureRollerMotor() {
        groundRollerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(45);
        groundRollerConfiguration.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        groundRollerConfiguration.closedLoop
            .pid(0.01, 0.0, 0.0);

        algeaGroundRollingEncoder.setPosition(0.0);
    
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground Pivoting Motor Encoder Position", algeaGroundPivotingEncoder.getPosition());
        SmartDashboard.putNumber("Ground Pivoting Motor Temperature", algeaGroundPivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Ground Target", groundPivotingState.getPosition());
        SmartDashboard.putNumber("Ground Roller Motor Velocity", algeaGroundPivotingEncoder.getVelocity());
        SmartDashboard.putNumber("Ground Roller Motor RPM", algeaGroundPivotingMotor.get());
        SmartDashboard.putNumber("Ground Roller Motor Temperature", algeaGroundPivotingMotor.getMotorTemperature());
    }

    public void setNeutralModes(IdleMode idleMode) {
        groundRollerConfiguration.idleMode(idleMode);
        groundPivotingConfiguration.idleMode(idleMode);

        algeaGroundRollerMotor.configure(groundRollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algeaGroundPivotingMotor.configure(groundPivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startRolling(double speed) {
        algeaGroundRollerMotor.set(speed);
    }

    public void stopRolling() {
        algeaGroundRollerMotor.set(-0.1);
    }

    public void startGroundPivot(double speed) {
        algeaGroundRollerMotor.set(speed);
    }

    public void stopGroundPivot() {
        algeaGroundRollerMotor.set(0.0);
    }

    public void setPivotingState(GroundPivotingState groundPivotingState) {
        this.groundPivotingState = groundPivotingState;
    }

    public GroundPivotingState getGroundPivotingState() {
        return groundPivotingState;
    }

    public void runToPosition() {
        double targetPosition = groundPivotingState.getPosition();
        groundPivotingPIDController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public boolean atSetpoint() {
        double targetPosition = groundPivotingState.getPosition();
        return (targetPosition - algeaGroundPivotingEncoder.getPosition() < 2 || targetPosition - algeaGroundPivotingEncoder.getPosition() > -2) ? true : false; 
    }
}