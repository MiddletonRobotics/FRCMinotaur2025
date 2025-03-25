package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.PhoenixUtil;
import frc.robot.utilities.Tunable;
import frc.robot.utilities.Tunable.TunableDouble;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem2 extends SubsystemBase {
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    private final TalonFXConfiguration elevatorConfiguration;
    private MotionMagicVoltage positionVoltageRequest;
    private VoltageOut voltageRequest;

    private TunableDouble kP = Tunable.doubleValue("Elevator/Gains/kP", 3.0);
    private TunableDouble kD = Tunable.doubleValue("Elevator/Gains/kD", 0.0);
    private TunableDouble kS = Tunable.doubleValue("Elevator/Gains/kS", 0.15);
    private TunableDouble kG = Tunable.doubleValue("Elevator/Gains/kG", 0.65);
    private TunableDouble kV = Tunable.doubleValue("Elevator/Gains/kV", 0.0);
    private TunableDouble kA = Tunable.doubleValue("Elevator/Gains/kA", 0.0);

    private TunableDouble maxAcceleration = Tunable.doubleValue("Elevator/Mex Acceleration", 8.0);
    private TunableDouble cruisingVelocity = Tunable.doubleValue("Elevator/Crusing Velocity", 8.0);
    private TunableDouble goalTolerence = Tunable.doubleValue("Elevator/Goal Tolerence", 0.02);

    private ElevatorStates currentElevatorState = ElevatorStates.STOW;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;

    private double positionGoalMeters;

    public ElevatorSubsystem2() {
        elevatorLeader = new TalonFX(Constants.ElevatorConstants.rightElevatorID);
        elevatorFollower = new TalonFX(Constants.ElevatorConstants.leftElevatorID);

        elevatorConfiguration = new TalonFXConfiguration();
        elevatorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfiguration.Slot0.kP = 3.2;
        elevatorConfiguration.Slot0.kD = 0.0;
        elevatorConfiguration.Slot0.kS = 0.8;
        elevatorConfiguration.Slot0.kG = 1.0;
        elevatorConfiguration.Slot0.kV = 0.0;
        elevatorConfiguration.Slot0.kA = 0.0;
        elevatorConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimit = 75;

        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = 5.0 / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio;
        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 5.0 / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio;

        elevatorLeader.getConfigurator().apply(elevatorConfiguration);
        elevatorFollower.getConfigurator().apply(elevatorConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));  

        elevatorLeader.optimizeBusUtilization();
        elevatorFollower.optimizeBusUtilization();

        positionVoltageRequest = new MotionMagicVoltage(0.0);
        voltageRequest = new VoltageOut(0.0);

        elevatorLeader.setPosition(0.0);
        elevatorFollower.setPosition(0.0);
        positionGoalMeters = 0.0;
        
        leadPosition = elevatorLeader.getPosition();
        followPosition = elevatorFollower.getPosition();
        leadVelocity = elevatorLeader.getVelocity();
        followVelocity = elevatorFollower.getVelocity();

        
        PhoenixUtil.run("Set Elevator Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                leadPosition,
                followPosition,
                leadVelocity,
                followVelocity
            )
        );
        PhoenixUtil.run("Set Elevator Signal Frequencies for Following", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                elevatorLeader.getDutyCycle(),
                elevatorLeader.getMotorVoltage(),
                elevatorLeader.getTorqueCurrent()
            )
        );
        PhoenixUtil.run("Optimize Elevator CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(20, elevatorLeader, elevatorFollower)
        );

    }

    public void setVoltage(double volts) {
        elevatorLeader.setControl(voltageRequest.withOutput(volts).withEnableFOC(false));
    }

    public void setSpeed(double speed) {
        elevatorLeader.set(speed);
        elevatorFollower.set(speed);
    }

    public void setElevatorState(ElevatorStates elevatorState) {
        this.currentElevatorState = elevatorState;
    }

    public void incrementElevatorState() {
        switch(currentElevatorState) {
            case STOW:
                currentElevatorState = ElevatorStates.L1;
                break;
            case L1:
                currentElevatorState = ElevatorStates.L2;
                break;
            case L2:
                currentElevatorState = ElevatorStates.L3;
                break;
            case L3:
                currentElevatorState = ElevatorStates.L4;
                break;
            case L4:
                currentElevatorState = ElevatorStates.BARGE;
                break;
            case BARGE:
                currentElevatorState = ElevatorStates.DEALGEAFIER_L2;
                break;
            case DEALGEAFIER_L2:
                currentElevatorState = ElevatorStates.DEALGEAFIER_L3;
                break;
            case DEALGEAFIER_L3:
                currentElevatorState = ElevatorStates.STOW;
                break;
            default:
                currentElevatorState = ElevatorStates.STOW;
        }
    }

    public void decrementElevatorState() {
        switch(currentElevatorState) {
            case STOW:
                currentElevatorState = ElevatorStates.DEALGEAFIER_L3;
                break;
            case DEALGEAFIER_L3:
                currentElevatorState = ElevatorStates.DEALGEAFIER_L2;
                break;
            case DEALGEAFIER_L2:
                currentElevatorState = ElevatorStates.BARGE;
                break;
            case BARGE:
                currentElevatorState = ElevatorStates.L4;
                break;
            case L4:
                currentElevatorState = ElevatorStates.L3;
                break;
            case L3:
                currentElevatorState = ElevatorStates.L2;
                break;
            case L2:
                currentElevatorState = ElevatorStates.L1;
                break;
            case L1:
                currentElevatorState = ElevatorStates.STOW;
                break;
            default:
                currentElevatorState = ElevatorStates.STOW;
        }
    }

    public ElevatorStates getElevatorState(ElevatorStates elevatorState) {
        return this.currentElevatorState;
    }

    public void setPosition(double meters) {
        elevatorLeader.setPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio) * 3);
    }

    public boolean isElevatorCooking() {
        return leadVelocity.getValueAsDouble() < 0.02;
    }

    public void setPositionGoal(double meters) {
        positionGoalMeters = meters;
        elevatorFollower.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
        elevatorLeader.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
    }

    public void runToPosition() {
        double meters = currentElevatorState.getPosition();
        elevatorFollower.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
        elevatorLeader.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
    }

    public boolean atGoal() {
        return Math.abs(positionGoalMeters - (leadPosition.getValueAsDouble() / ElevatorConstants.ElevatorGearRatio) * (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter)) <= 0.04;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(leadPosition, followPosition, leadVelocity, followVelocity);
        positionGoalMeters = this.currentElevatorState.getPosition();

        SmartDashboard.putBoolean("Elevator At Goal", atGoal());
        SmartDashboard.putBoolean("Elevator Stuck", isElevatorCooking());
        SmartDashboard.putNumber("Elevator Target Postion", (positionGoalMeters  / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio));
        SmartDashboard.putNumber("Elevator Current Position", leadPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current Meters", (leadPosition.getValueAsDouble() / ElevatorConstants.ElevatorGearRatio) * (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter));
        SmartDashboard.putNumber("Elevator Follower Position", followPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Error", (positionGoalMeters  / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio) - leadPosition.getValueAsDouble());
    }
}
