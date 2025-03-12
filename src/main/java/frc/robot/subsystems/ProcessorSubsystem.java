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
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ProcessorSubsystem extends SubsystemBase {
    private SparkMax pivotingMotor;
    private SparkMax rollerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig rollerConfiguration;

    private RelativeEncoder pivotingEncoder;
    private RelativeEncoder rollingEncoder;
    private ProfiledPIDController pidController;

    private ArmFeedforward feedforward = new ArmFeedforward(
        Constants.ProcessorConstants.kS.in(Volts),
        Constants.ProcessorConstants.kG.in(Volts),
        Constants.ProcessorConstants.kV
    );   

    public enum GroundPivotingState {
        STORED(Degrees.of(0.0)),
        GROUND(Degrees.of(56.0));

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
        pivotingConfiguration = new SparkMaxConfig();
        configurePivotingMotor();

        rollerMotor = new SparkMax(Constants.ProcessorConstants.rollerMotorID, MotorType.kBrushless);
        rollingEncoder = rollerMotor.getEncoder();
        rollerConfiguration = new SparkMaxConfig();
        configureRollerMotor();

        pidController = new ProfiledPIDController(
            0.1,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                Constants.ProcessorConstants.LimitedVelocity.in(RadiansPerSecond),
                Constants.ProcessorConstants.LimitedAcceleration.in(RadiansPerSecondPerSecond)
            )
        );

        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(
            Constants.ProcessorConstants.MaximumAllowedPositionError.in(Radians), 
            Constants.ProcessorConstants.MaximumAllowedVelocityError.in(RadiansPerSecond)
        );
    }

    public void configurePivotingMotor() {
        pivotingConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(35);
        pivotingConfiguration.encoder
            .positionConversionFactor(Constants.ProcessorConstants.PositionConversionFactor)
            .velocityConversionFactor(Constants.ProcessorConstants.VelocityConversionFactor);

        pivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotingEncoder.setPosition(0.0);
    }

    public void configureRollerMotor() {
        rollerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(55);
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
        SmartDashboard.putNumber("Processor Pivot Target", groundPivotingState.getPosition().in(Degrees));
        SmartDashboard.putNumber("Processor Roller Velocity", pivotingEncoder.getVelocity());
        SmartDashboard.putNumber("Processor Roller AO", pivotingMotor.get());
        SmartDashboard.putNumber("Processor Roller Temp.", pivotingMotor.getMotorTemperature());
        SmartDashboard.putNumber("Processor Pivot Stall", getRollerStall());
    }

    public void setNeutralModes(IdleMode idleMode) {
        rollerConfiguration.idleMode(idleMode);
        pivotingConfiguration.idleMode(idleMode);

        rollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotingMotor.configure(pivotingConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getPivotPosition() {
        return pivotingEncoder.getPosition();
    }

    public State getPIDSetpoint() {
        return pidController.getSetpoint();
    }

    public void startRolling(double speed) {
        rollerMotor.set(speed);
    }

    public void stopRolling() {
        rollerMotor.set(-0.1);
    }

    public void startGroundPivot(double speed) {
        pivotingMotor.set(speed);
    }

    public void stopGroundPivot() {
        pivotingMotor.set(0.0);
    }

    public void setPivotingState(GroundPivotingState groundPivotingState) {
        this.groundPivotingState = groundPivotingState;
    }

    public GroundPivotingState getGroundPivotingState() {
        return groundPivotingState;
    }

    public double getRollerStall() {
        return rollerMotor.getOutputCurrent();
    }

    public boolean isRollerCooking() {
        double stall = getRollerStall();

        if(stall > 15) {
            return true;
        } else {
            return false;
        }
    }

    public void runToPosition() {
        Angle targetPosition = groundPivotingState.getPosition();
        pivotingMotor.set(pidController.calculate(getPivotPosition(), targetPosition.in(Radians)) + feedforward.calculate(targetPosition.in(Radians), (getPIDSetpoint().velocity) / RobotController.getBatteryVoltage()));
    }

    public boolean atSetpoint() {
        return pidController.atGoal();
    }
}