package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.DealgeafierSubsystem.PivotingState;
import frc.robot.utilities.constants.Constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import java.util.Set;

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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ProcessorSubsystem extends SubsystemBase {
    private SparkMax pivotingMotor;
    private SparkMax rollerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig rollerConfiguration;

    private RelativeEncoder pivotingEncoder;
    private RelativeEncoder rollingEncoder;
    private SparkClosedLoopController pivotPIDController;

    private Alert pivotDisconnected;
    private Alert rollerDisconnected;
    private Alert deviceBrownedOut;

    private ArmFeedforward feedforward = new ArmFeedforward(
        Constants.ProcessorConstants.kS.in(Volts),
        Constants.ProcessorConstants.kG.in(Volts),
        Constants.ProcessorConstants.kV
    );   

    public enum GroundPivotingState {
        STORED(Degrees.of(91.6)),
        INTAKEN(Degrees.of(106.0)),
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

        pivotDisconnected = new Alert("Processor Pivot CAN Issue", AlertType.kError);
        rollerDisconnected = new Alert("Processor Roller CAN Issue", AlertType.kError);
        deviceBrownedOut = new Alert("Dealgeafier Hardware Browned Out", AlertType.kError);
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
        SmartDashboard.putNumber("Processor Pivot Position", pivotingEncoder.getPosition());
        SmartDashboard.putNumber("Processor Pivot Motor Temp.", pivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Processor Pivot Target", groundPivotingState.getPosition().in(Radians));
        SmartDashboard.putNumber("Processor Roller Velocity", pivotingEncoder.getVelocity());
        SmartDashboard.putNumber("Processor Roller Temp.", pivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Processor Roller Stall", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Processor Roller Cooking", isRollerCooking());
        SmartDashboard.putNumber("Processor Pivot Error", calculateError());
        SmartDashboard.putBoolean("Processor At Goal", atGoal());

        pivotDisconnected.set(pivotingMotor.getFaults().can);
        rollerDisconnected.set(rollerMotor.getFaults().can);
        deviceBrownedOut.set(isBrownedOut());
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
        rollerMotor.set(-0.05);
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