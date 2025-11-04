package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.PhoenixUtil;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    private final TalonFXConfiguration elevatorConfiguration;
    private MotionMagicVoltage positionVoltageRequest;
    private VoltageOut voltageRequest;

    private ElevatorStates currentElevatorState = ElevatorStates.STOW;

    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;
    private final StatusSignal<Temperature> leadTempurature;
    private final StatusSignal<Temperature> followTempurature;

    private final StatusSignal<Boolean> leadTempuratureFault;
    private final StatusSignal<Boolean> followTempuratureFault;
    private final StatusSignal<Boolean> leadOverVoltageFault;
    private final StatusSignal<Boolean> followOverVoltageFault;
    private final StatusSignal<Boolean> leadSyncCANCoderFault;
    private final StatusSignal<Boolean> followSyncCANCoderFault;

    private final Debouncer connectedDebouncer = new Debouncer(0.5);
    private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

    private Alert elevatorLeaderCANDisconnected;
    private Alert elevatorLeaderOverTempurature;
    private Alert elevatorLeaderOverCurrent;
    private Alert elevatorLeaderFeature;

    private Alert elevatorFollowerCANDisconnected;
    private Alert elevatorFollowerOverTempurature;
    private Alert elevatorFollowerOverCurrent;
    private Alert elevatorFollowerFeature;

    private double positionGoalMeters;

    public ElevatorSubsystem() {
        elevatorLeader = new TalonFX(Constants.ElevatorConstants.rightElevatorID, "*");
        elevatorFollower = new TalonFX(Constants.ElevatorConstants.leftElevatorID, "*");

        elevatorConfiguration = new TalonFXConfiguration();
        elevatorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfiguration.Slot0.kP = 3.4;
        elevatorConfiguration.Slot0.kD = 0.0;
        elevatorConfiguration.Slot0.kS = 0.9;
        elevatorConfiguration.Slot0.kG = 1.05;
        elevatorConfiguration.Slot0.kV = 0.0;
        elevatorConfiguration.Slot0.kA = 0.0;
        elevatorConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfiguration.CurrentLimits.StatorCurrentLimit = 90;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimit = 90;

        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = 5.0 / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio;
        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 5.0 / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio;

        elevatorLeader.getConfigurator().apply(elevatorConfiguration);
        elevatorFollower.getConfigurator().apply(elevatorConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));  

        positionVoltageRequest = new MotionMagicVoltage(0.0);
        voltageRequest = new VoltageOut(0.0);

        elevatorLeader.setPosition(0.0);
        elevatorFollower.setPosition(0.0);
        positionGoalMeters = 0.0;
        
        leadPosition = elevatorLeader.getPosition();
        followPosition = elevatorFollower.getPosition();
        leadVelocity = elevatorLeader.getVelocity();
        followVelocity = elevatorFollower.getVelocity();
        leadTempurature = elevatorLeader.getDeviceTemp();
        followTempurature = elevatorFollower.getDeviceTemp();

        leadOverVoltageFault = elevatorLeader.getFault_OverSupplyV();
        followOverVoltageFault = elevatorFollower.getFault_OverSupplyV();

        leadTempuratureFault = elevatorLeader.getFault_DeviceTemp();
        followTempuratureFault = elevatorFollower.getFault_DeviceTemp();

        leadSyncCANCoderFault = elevatorLeader.getFault_UsingFusedCANcoderWhileUnlicensed();
        followSyncCANCoderFault = elevatorFollower.getFault_UsingFusedCANcoderWhileUnlicensed();

        BaseStatusSignal.setUpdateFrequencyForAll(
            200,
            leadPosition,
            followPosition,
            leadVelocity,
            followVelocity,
            leadTempurature,
            followTempurature,
            leadOverVoltageFault,
            followOverVoltageFault,
            leadTempuratureFault, 
            followTempuratureFault,
            leadSyncCANCoderFault,
            followSyncCANCoderFault
        );

        /* 

        BaseStatusSignal.setUpdateFrequencyForAll(
            500,
            elevatorLeader.getDutyCycle(),
            elevatorLeader.getMotorVoltage(),
            elevatorLeader.getTorqueCurrent()
        );

        */

        PhoenixUtil.run("Optimize Elevator CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(20, elevatorLeader, elevatorFollower)
        );

        elevatorLeaderCANDisconnected = new Alert("Elevator Leader CAN Disconnect. Will not function.", AlertType.kError);
        elevatorLeaderOverTempurature = new Alert("Elevator Leader Over Tempurature", AlertType.kWarning);
        elevatorLeaderOverCurrent = new Alert("Elevator Leader Over Current", AlertType.kWarning);
        elevatorLeaderFeature = new Alert("Elevator Leader is not lisensed, reverting to RemoteCANCoder.", AlertType.kWarning);

        elevatorFollowerCANDisconnected = new Alert("Elevator Follower CAN Disconnect. Will not function.", AlertType.kError);
        elevatorFollowerOverTempurature = new Alert("Elevator Follower Over Tempurature", AlertType.kWarning);
        elevatorFollowerOverCurrent = new Alert("Elevator Follower Over Current", AlertType.kWarning);
        elevatorFollowerFeature = new Alert("Elevator Follower is not lisensed, reverting to RemoteCANCoder.", AlertType.kWarning);
    }

    public void setVoltage(double volts) {
        elevatorLeader.setControl(voltageRequest.withOutput(volts).withEnableFOC(false));
    }

    public double elevatorCurrentPosition() {
        return (leadPosition.getValueAsDouble() / ElevatorConstants.ElevatorGearRatio) * (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter);
    }

    public void setSpeed(double speed) {
        elevatorLeader.set(speed);
        elevatorFollower.set(speed);
    }

    public void resetEncoders() {
        elevatorLeader.setPosition(0.0);
        elevatorFollower.setPosition(0.0);
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

    public ElevatorStates getElevatorState() {
        return this.currentElevatorState;
    }

    public void setPosition(double meters) {
        elevatorLeader.setPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio) * 3);
    }

    public boolean isElevatorCooking() {
        return leadVelocity.getValueAsDouble() < 0.02 && elevatorLeader.getMotorStallCurrent().getValue().in(Amps) > Amps.of(12.0).baseUnitMagnitude();
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

    public void runToStow() {
        double meters = ElevatorStates.STOW.getPosition();
        elevatorFollower.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
        elevatorLeader.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
    }

    public void runToL1() {
        double meters = ElevatorStates.L1.getPosition();
        elevatorFollower.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
        elevatorLeader.setControl(positionVoltageRequest.withPosition((meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio)));
    }

    public boolean atGoal() {
        return Math.abs(positionGoalMeters - (leadPosition.getValueAsDouble() / ElevatorConstants.ElevatorGearRatio) * (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter)) <= 0.04;
    }

    @Override
    public void periodic() {
        updateLogs();
        updateAlerts();
        
        BaseStatusSignal.refreshAll(leadPosition, followPosition, leadVelocity, followVelocity, leadTempurature, followTempurature);
        positionGoalMeters = this.currentElevatorState.getPosition();
    }

    public void updateLogs() {
        SmartDashboard.putBoolean("Elevator At Goal", atGoal());
        SmartDashboard.putBoolean("Elevator Stuck", isElevatorCooking());
        SmartDashboard.putString("Elevator State: ", currentElevatorState.toString());
        SmartDashboard.putNumber("Elevator Target Postion", (positionGoalMeters  / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio));
        SmartDashboard.putNumber("Elevator Current Position", leadPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current Meters", (leadPosition.getValueAsDouble() / ElevatorConstants.ElevatorGearRatio) * (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter));
        SmartDashboard.putNumber("Elevator Follower Position", followPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Error", (positionGoalMeters  / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio) - leadPosition.getValueAsDouble());
        SmartDashboard.putNumber("Elevator Lead Temp", leadTempurature.getValue().in(Celsius));
        SmartDashboard.putNumber("Elevator Follow Temp", followTempurature.getValue().in(Celsius)); 
    }

    public void updateAlerts() {
        elevatorLeaderCANDisconnected.set(!connectedDebouncer.calculate(elevatorLeader.isAlive()));
        elevatorLeaderOverTempurature.set(leadTempuratureFault.getValue().booleanValue());
        elevatorLeaderOverCurrent.set(leadOverVoltageFault.getValue().booleanValue());
        elevatorLeaderFeature.set(leadSyncCANCoderFault.getValue().booleanValue());

        elevatorFollowerCANDisconnected.set(!followerConnectedDebouncer.calculate(elevatorFollower.isAlive()));
        elevatorFollowerOverTempurature.set(followTempuratureFault.getValue().booleanValue());
        elevatorFollowerOverCurrent.set(followOverVoltageFault.getValue().booleanValue());
        elevatorFollowerFeature.set(followSyncCANCoderFault.getValue().booleanValue());
    }
}
