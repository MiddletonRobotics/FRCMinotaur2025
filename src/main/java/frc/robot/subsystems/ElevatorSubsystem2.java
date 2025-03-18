package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.utilities.TunableNumber;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;

public class ElevatorSubsystem2 {
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    private final TalonFXConfiguration elevatorConfiguration;
    private MotionMagicVoltage positionVoltageRequest;
    private VoltageOut voltageRequest;

    private TunableNumber kP = new TunableNumber("Elevator/Gains/kP", 0.1);
    private TunableNumber kD = new TunableNumber("Elevator/Gains/kD", 0.0);
    private TunableNumber kS = new TunableNumber("Elevator/Gains/kS", 0.0);
    private TunableNumber kG = new TunableNumber("Elevator/Gains/kG", 0.0);
    private TunableNumber kV = new TunableNumber("Elevator/Gains/kV", 0.0);
    private TunableNumber kA = new TunableNumber("Elevator/Gains/kA", 0.0);

    private TunableNumber maxAcceleration = new TunableNumber("Elevator/Mex Acceleration", 8.0);
    private TunableNumber cruisingVelocity = new TunableNumber("Elevator/Crusing Velocity", 8.0);
    private TunableNumber goalTolerence = new TunableNumber("Elevator/Goal Tolerence", 0.02);

    private double positionGoalMeters;

    public ElevatorSubsystem2() {
        elevatorLeader = new TalonFX(Constants.ElevatorConstants.rightElevatorID);
        elevatorFollower = new TalonFX(Constants.ElevatorConstants.leftElevatorID);

        elevatorConfiguration = new TalonFXConfiguration();
        elevatorConfiguration.Slot0.kP = kP.get();
        elevatorConfiguration.Slot0.kD = kD.get();
        elevatorConfiguration.Slot0.kS = kS.get();
        elevatorConfiguration.Slot0.kG = kG.get();
        elevatorConfiguration.Slot0.kV = kV.get();
        elevatorConfiguration.Slot0.kG = kG.get();
        elevatorConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimit = 50;

        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration.get() / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio;
        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruisingVelocity.get() / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio;

        elevatorLeader.getConfigurator().apply(elevatorConfiguration);
        elevatorLeader.getConfigurator().apply(elevatorConfiguration);
        elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), true));

        positionGoalMeters = 0.0;

        elevatorLeader.optimizeBusUtilization();
        elevatorFollower.optimizeBusUtilization();

        positionVoltageRequest = new MotionMagicVoltage(0.0);
        voltageRequest = new VoltageOut(0.0);
    }

    public void setVoltage(double volts) {
        elevatorLeader.setControl(voltageRequest.withOutput(volts).withEnableFOC(false));
    }

    public void setPosition(double meters) {
        elevatorLeader.setPosition(meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio);
    }

    public void setPositionGoal(double meters) {
        positionGoalMeters = meters;
        elevatorLeader.setControl(positionVoltageRequest.withPosition(meters / (2 * Math.PI * ElevatorConstants.SprocketPitchDiameter) * ElevatorConstants.ElevatorGearRatio));
    }

    public boolean atGoal() {
        return Math.abs(positionGoalMeters - elevatorLeader.getPosition().getValueAsDouble()) <= goalTolerence.get();
    }
}
