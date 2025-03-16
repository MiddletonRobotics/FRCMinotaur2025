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
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.constants.Constants;

public class DealgeafierSubsystem extends SubsystemBase {
    private SparkMax pivotingMotor;
    private SparkMax rollerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig rollerConfiguration;

    private RelativeEncoder pivotingEncoder;
    private RelativeEncoder rollingEncoder;
    private SparkClosedLoopController pivotingPIDController;

    private Alert pivotDisconnected;
    private Alert rollerDisconnected;
    private Alert deviceBrownedOut;

    private ArmFeedforward feedforward = new ArmFeedforward(
        Constants.DealgeafierConstants.kS.in(Volts),
        Constants.DealgeafierConstants.kG.in(Volts),
        Constants.DealgeafierConstants.kV
    );

    private DigitalInput algeaLimitSwitch;

    public enum PivotingState {
        START(Degrees.of(78.5)),
        STORED(Degrees.of(90.0)),
        BARGE(Degrees.of(115.0)),
        REEF(Degrees.of(190.0)),
        GROUND(Degrees.of(240.0));

        private final Angle position;

        private PivotingState(Angle position) {
            this.position = position;
        }

        public Angle getPosition() {
            return this.position;
        }
    }

    public PivotingState pivotingState = PivotingState.START;

    public DealgeafierSubsystem() {
        pivotingMotor = new SparkMax(Constants.DealgeafierConstants.pivotingMotorID, MotorType.kBrushless);
        pivotingEncoder = pivotingMotor.getEncoder();
        pivotingConfiguration = new SparkMaxConfig();
        pivotingPIDController = pivotingMotor.getClosedLoopController();
        configurePivotingMotor();

        rollerMotor = new SparkMax(Constants.DealgeafierConstants.rollerMotorID, MotorType.kBrushless);
        rollingEncoder = rollerMotor.getEncoder();
        rollerConfiguration = new SparkMaxConfig();
        configureRollerMotor();

        pivotDisconnected = new Alert("Dealgeafier Pivot CAN Issue", AlertType.kError);
        rollerDisconnected = new Alert("Dealgeafier Roller CAN Issue", AlertType.kError);
        deviceBrownedOut = new Alert("Dealgeafier Hardware Browned Out", AlertType.kError);

        algeaLimitSwitch = new DigitalInput(3); // Initialize limit switch on DIO port 0
    }

    public void configurePivotingMotor() {
        pivotingConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(35);
        pivotingConfiguration.encoder
            .positionConversionFactor(Constants.DealgeafierConstants.PositionConversionFactor)
            .velocityConversionFactor(Constants.DealgeafierConstants.VelocityConversionFactor);
        pivotingConfiguration.closedLoop
            .pid(0.25, 0.0, 0.0);

        pivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotingEncoder.setPosition(pivotingState.getPosition().in(Degrees));
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

        rollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollingEncoder.setPosition(0.0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Dealgeafier Pivot Position", pivotingEncoder.getPosition());
        SmartDashboard.putNumber("Dealgeafier Pivot Temp.", pivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Dealgeafier Pivot Target", pivotingState.getPosition().in(Degrees));
        SmartDashboard.putNumber("Dealgeafier Roller Vel.", rollingEncoder.getVelocity());
        SmartDashboard.putNumber("Dealgeafier Roller AO", pivotingMotor.get());
        SmartDashboard.putNumber("Dealgeafier Motor Temp.", rollerMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Dealgeafier Limit- Switch", getLimitSwitch());
        SmartDashboard.putNumber("Dealgeafier Pivot Error", calculateError());
        SmartDashboard.putBoolean("At Goal", atGoal());

        pivotDisconnected.set(pivotingMotor.getFaults().can);
        rollerDisconnected.set(rollerMotor.getFaults().can);
        deviceBrownedOut.set(isBrownedOut());
    }

    public void setNeutralModes(IdleMode idleMode) {
        rollerConfiguration.idleMode(idleMode);
        pivotingConfiguration.idleMode(idleMode);

        rollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double calculateError() {
        Angle targetPosition = getPivotingState().getPosition();
        return targetPosition.in(Radians) - pivotingEncoder.getPosition();
    }

    public boolean getLimitSwitch() {
        return !algeaLimitSwitch.get();
    }

    public void startRolling(double speed) {
        rollerMotor.set(speed);
    }

    public void stopRolling() {
        rollerMotor.set(-0.1);
    }

    public void startPivot(double speed) {
        pivotingMotor.set(speed);
    }

    public void stopPivot() {
        pivotingMotor.set(0.0);
    }

    public void setPivotingState(PivotingState pivotingState) {
        this.pivotingState = pivotingState;
    }

    public PivotingState getPivotingState() {
        return pivotingState;
    }

    public void runToPosition() {
        Angle targetPosition = pivotingState.getPosition();
        pivotingPIDController.setReference(targetPosition.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(pivotingEncoder.getPosition(), 0.15), ArbFFUnits.kVoltage);
    }

    public boolean atGoal() {
        Angle targetPosition = getPivotingState().getPosition();
        return (targetPosition.in(Radians) - pivotingEncoder.getPosition() < 0.1 && targetPosition.in(Radians) - pivotingEncoder.getPosition() > -0.1);
    }

    public boolean isBrownedOut() {
        return pivotingMotor.getStickyWarnings().brownout || rollerMotor.getStickyWarnings().brownout;
    }
}
