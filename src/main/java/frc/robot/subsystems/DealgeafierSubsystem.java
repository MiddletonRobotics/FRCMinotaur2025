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
import edu.wpi.first.wpilibj.Timer;
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

    private ProfiledPIDController profiledPIDController;
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;

    private final TrapezoidProfile.State currentAngle = new TrapezoidProfile.State();
    private final TrapezoidProfile.State goalAngle = new TrapezoidProfile.State();
    private final Timer timer = new Timer();

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
        START(Degrees.of(71.0)),
        STORED(Degrees.of(90.0)),
        BARGE(Degrees.of(107.0)),
        REEF(Degrees.of(190.0)),
        GROUND(Degrees.of(235.0));

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
        pivotingMotor = new SparkMax(Constants.DealgeafierConstants.pivotingMotorID, MotorType.kBrushless);
        pivotingEncoder = pivotingMotor.getEncoder();
        pivotingConfiguration = new SparkMaxConfig();
        pivotingPIDController = pivotingMotor.getClosedLoopController();
        configurePivotingMotor();

        rollerMotor = new SparkMax(Constants.DealgeafierConstants.rollerMotorID, MotorType.kBrushless);
        rollingEncoder = rollerMotor.getEncoder();
        rollerConfiguration = new SparkMaxConfig();
        configureRollerMotor();

        constraints = new TrapezoidProfile.Constraints(1.25, 0.25);
        profile = new TrapezoidProfile(constraints);
        profiledPIDController = new ProfiledPIDController(0.4, 0.0, 0.0, constraints);

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
            .pid(0.3, 0.0, 0.0);

        pivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotingEncoder.setPosition(PivotingState.STORED.getPosition().in(Radians));
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
        SmartDashboard.putNumber("Dealgeafier Pivot Target", pivotingState.getPosition().in(Radians));
        SmartDashboard.putNumber("Dealgeafier Roller Vel.", rollingEncoder.getVelocity());
        SmartDashboard.putNumber("Dealgeafier Roller AO", pivotingMotor.get());
        SmartDashboard.putNumber("Dealgeafier Motor Temp.", rollerMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Dealgeafier Limit- Switch", getLimitSwitch());
        SmartDashboard.putNumber("Dealgeafier Pivot Error", calculateError());
        SmartDashboard.putBoolean("At Goal", atTargetPosition());

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

    public boolean getLimitSwitch() {
        return !algeaLimitSwitch.get();
    }

    public PivotingState getPivotingState() {
        return pivotingState;
    }

    public void setPivotingState(PivotingState pivotingState) {
        this.pivotingState = pivotingState;
        goalAngle.position = pivotingState.getPosition().in(Radians);
        timer.reset();
        timer.start();
    }

    public double calculateError() {
        Angle targetPosition = getPivotingState().getPosition();
        return targetPosition.in(Radians) - pivotingEncoder.getPosition();
    }

    public void runToPosition() {
        Angle targetPosition = pivotingState.getPosition();
        pivotingPIDController.setReference(targetPosition.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(targetPosition.in(Radians), 0.15), ArbFFUnits.kVoltage);
    }

    public void runToProfiledPosition() {
        TrapezoidProfile.State desiredState = profile.calculate(timer.get(), currentAngle, goalAngle);
        double ff = feedforward.calculate(desiredState.position, desiredState.velocity);
        double feedback = profiledPIDController.calculate(currentAngle.position, desiredState);
        double voltage = ff + feedback;
        pivotingMotor.setVoltage(voltage);
    }

    public boolean atTargetPosition() {
        Angle targetPosition = getPivotingState().getPosition();
        return (targetPosition.in(Radians) - pivotingEncoder.getPosition() < 0.1 && targetPosition.in(Radians) - pivotingEncoder.getPosition() > -0.1);
    }

    public boolean profileAtGoal() {
        return profiledPIDController.atGoal();
    }

    public boolean isBrownedOut() {
        return pivotingMotor.getStickyWarnings().brownout || rollerMotor.getStickyWarnings().brownout;
    }

    public void startRolling(double speed) {
        rollerMotor.set(speed);
    }

    public void stopRolling() {
        rollerMotor.set(-0.5);
    }

    public void startPivot(double speed) {
        pivotingMotor.set(speed);
    }

    public void stopPivot() {
        pivotingMotor.set(0.0);
    }
}
