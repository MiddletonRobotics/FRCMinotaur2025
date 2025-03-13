package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.constants.Constants;

public class DealgeafierSubsystem extends SubsystemBase {
    private SparkMax algeaElevatorPivotingMotor;
    private SparkMax algeaElevatorRollerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig rollerConfiguration;

    private RelativeEncoder algeaElevatorPivotingEncoder;
    private RelativeEncoder algeaElevatorRollingEncoder;
    private ProfiledPIDController pidController;

    private ArmFeedforward feedforward = new ArmFeedforward(
        Constants.DealgeafierConstants.kS.in(Volts),
        Constants.DealgeafierConstants.kG.in(Volts),
        Constants.DealgeafierConstants.kV
    );

    private DigitalInput algeaLimitSwitch;

    public enum PivotingState {
        STORED(Degrees.of(0.0)),
        BARGE(Degrees.of(0.0)),
        REEF(Degrees.of(0.0)),
        GROUND(Degrees.of(0.0));

        private final Angle position;

        private PivotingState(Angle position) {
            this.position = position;
        }

        public Angle getPosition() {
            return this.position;
        }
    }

    public PivotingState pivotingState = PivotingState.STORED;

    public DealgeafierSubsystem() {
        algeaElevatorPivotingMotor = new SparkMax(Constants.DealgeafierConstants.pivotingMotorID, MotorType.kBrushless);
        algeaElevatorPivotingEncoder = algeaElevatorPivotingMotor.getEncoder();
        pivotingConfiguration = new SparkMaxConfig();
        configurePivotingMotor();

        algeaElevatorRollerMotor = new SparkMax(Constants.DealgeafierConstants.rollerMotorID, MotorType.kBrushless);
        algeaElevatorRollingEncoder = algeaElevatorRollerMotor.getEncoder();
        rollerConfiguration = new SparkMaxConfig();
        configureRollerMotor();

        pidController = new ProfiledPIDController(
            0.001,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                Constants.DealgeafierConstants.LimitedVelocity.in(RadiansPerSecond),
                Constants.DealgeafierConstants.LimitedAcceleration.in(RadiansPerSecondPerSecond)
            )
        );

        pidController.enableContinuousInput(0, 2 * Math.PI);
        pidController.setTolerance(
            Constants.DealgeafierConstants.MaximumAllowedPositionError.in(Radians), 
            Constants.DealgeafierConstants.MaximumAllowedVelocityError.in(RadiansPerSecond)
        );

        algeaLimitSwitch = new DigitalInput(0); // Initialize limit switch on DIO port 0
    }

    public void configurePivotingMotor() {
        pivotingConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(35);
        pivotingConfiguration.encoder
            .positionConversionFactor(Constants.DealgeafierConstants.PositionConversionFactor)
            .velocityConversionFactor(Constants.DealgeafierConstants.VelocityConversionFactor);

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
        //SmartDashboard.putNumber("Dealgeafier Pivot Position", algeaElevatorPivotingEncoder.getPosition());
        //SmartDashboard.putNumber("Dealgeafier Pivot Temp.", algeaElevatorPivotingMotor.getMotorTemperature());
        //SmartDashboard.putNumber("Dealgeafier Pivot Target", pivotingState.getPosition().in(Degrees));
        //SmartDashboard.putNumber("Dealgeafier Roller Vel.", algeaElevatorRollingEncoder.getVelocity());
        //SmartDashboard.putNumber("Dealgeafier Roller AO", algeaElevatorPivotingMotor.get());
        //SmartDashboard.putNumber("Dealgeafier Motor Temp.", algeaElevatorRollerMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Dealgeafier Limit Switch", getLimitSwitch());
        //SmartDashboard.putBoolean("At Goal", atSetpoint());
    }

    public void setNeutralModes(IdleMode idleMode) {
        rollerConfiguration.idleMode(idleMode);
        pivotingConfiguration.idleMode(idleMode);

        algeaElevatorRollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algeaElevatorPivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getPivotPosition() {
        return algeaElevatorPivotingEncoder.getPosition();
    }

    public boolean getLimitSwitch() {
        return !algeaLimitSwitch.get();
    }

    public State getPIDSetpoint() {
        return pidController.getSetpoint();
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
        Angle targetPosition = pivotingState.getPosition();
        algeaElevatorPivotingMotor.set(pidController.calculate(getPivotPosition(), targetPosition.in(Radians)) + feedforward.calculate(targetPosition.in(Radians), getPIDSetpoint().velocity / RobotController.getBatteryVoltage()));
    }

    public boolean atSetpoint() {
        return pidController.atGoal();
    }
}
