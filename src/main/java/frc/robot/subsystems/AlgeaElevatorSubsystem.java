package frc.robot.subsystems;

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

public class AlgeaElevatorSubsystem extends SubsystemBase {
    private SparkMax algeaElevatorPivotingMotor;
    private SparkMax algeaElevatorRollerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig rollerConfiguration;

    private RelativeEncoder algeaElevatorPivotingEncoder;
    private RelativeEncoder algeaElevatorRollingEncoder;

    private SparkClosedLoopController pivotingPIDController;
    private SparkClosedLoopController rollerPIDController;

    private DigitalInput algeaLimitSwitch;

    private ShuffleboardTab logger;

    public enum PivotingState {
        STORED(0),
        BARGE(0),
        REEF(31.619),
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
        pivotingConfiguration = new SparkMaxConfig();
        configurePivotingMotor();

        algeaElevatorRollerMotor = new SparkMax(20, MotorType.kBrushless);
        algeaElevatorRollingEncoder = algeaElevatorRollerMotor.getEncoder();
        rollerPIDController = algeaElevatorRollerMotor.getClosedLoopController();
        rollerConfiguration = new SparkMaxConfig();
        configureRollerMotor();

        algeaLimitSwitch = new DigitalInput(0); // Initialize limit switch on DIO port 0
        logger = Shuffleboard.getTab(getName());
    }

    public void configurePivotingMotor() {
        pivotingConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(35);
        pivotingConfiguration.encoder
            .positionConversionFactor(360 / (25 * (42 / 36)))
            .velocityConversionFactor(1);
        pivotingConfiguration.closedLoop
            .pid(0.03, 0.0, 0.0);

        algeaElevatorPivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algeaElevatorPivotingEncoder.setPosition(0.0);
    }

    public void configureRollerMotor() {
        rollerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(32);
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
        SmartDashboard.putNumber("Pivoting Motor Encoder Position", algeaElevatorPivotingEncoder.getPosition());
        SmartDashboard.putNumber("Pivoting Motor Temperature", algeaElevatorPivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Target", pivotingState.getPosition());
        SmartDashboard.putNumber("Roller Motor Velocity", algeaElevatorRollingEncoder.getVelocity());
        SmartDashboard.putNumber("Roller Motor RPM", algeaElevatorRollerMotor.get());
        SmartDashboard.putNumber("Roller Motor Temperature", algeaElevatorRollerMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Algea Limit Switch", firstLimitBroken());
    }

    public void setNeutralModes(IdleMode idleMode) {
        rollerConfiguration.idleMode(idleMode);
        pivotingConfiguration.idleMode(idleMode);

        algeaElevatorRollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algeaElevatorPivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startRolling(double speed) {
        algeaElevatorRollerMotor.set(speed);
    }

    public void stopRolling() {
        algeaElevatorRollerMotor.set(-0.1);
    }

    public void setPivotingState(PivotingState pivotingState) {
        this.pivotingState = pivotingState;
    }

    public PivotingState getPivotingState() {
        return pivotingState;
    }

    public void runToPosition() {
        double targetPosition = pivotingState.getPosition();
        pivotingPIDController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public boolean atSetpoint() {
        double targetPosition = pivotingState.getPosition();
        return (targetPosition - algeaElevatorPivotingEncoder.getPosition() < 2 || targetPosition - algeaElevatorPivotingEncoder.getPosition() > -2) ? true : false; 
    }

    public boolean firstLimitBroken() {
        return !algeaLimitSwitch.get();
    }

    public Command intakeUntilBroken(double speed) {
        return startEnd(() -> startRolling(speed), () -> stopRolling()).until(() -> firstLimitBroken());
      }
}
