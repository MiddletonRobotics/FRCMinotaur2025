//Import all that jazz you need for swerve drive coding.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.SwerveConstants;
import frc.robot.utilities.constants.SwerveModuleConstants;

//Sets up swerve drive class with encoders. This section can and should be added to.
public class SwerveModule extends SubsystemBase {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SwerveModuleState expectedState = new SwerveModuleState();

    private TalonFX driveMotor;
    private TalonFX steeringMotor;
    private CANcoder swerveEncoder;

    private BaseStatusSignal[] signals;
    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Angle> steerPosition;
    private StatusSignal<AngularVelocity> steerVelocity;
    private SwerveModulePosition internalState = new SwerveModulePosition();

    private DutyCycleOut driveDutyCyleRequest = new DutyCycleOut(0);
    private VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private MotionMagicVoltage steerPositionRequest = new MotionMagicVoltage(0);

    private CANcoderConfigurator swerveEncoderConfigurator;
    private TalonFXConfiguration driveMotorConfiguration, steerMotorConfiguration;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.ModuleConstants.TalonFXModuleConstants.driveKS, 
        Constants.ModuleConstants.TalonFXModuleConstants.driveKV, 
        Constants.ModuleConstants.TalonFXModuleConstants.driveKA
    );

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        swerveEncoder = new CANcoder(moduleConstants.swerveEncoderID);
        driveMotorConfiguration = new TalonFXConfiguration();
        configureSwerveEncoder();

        steeringMotor = new TalonFX(moduleConstants.angleMotorID);
        steerMotorConfiguration = new TalonFXConfiguration();
        configureSteeringMotor(moduleConstants);

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configureDriveMotor(moduleConstants);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = steeringMotor.getPosition();
        steerVelocity = steeringMotor.getVelocity();

        signals = new BaseStatusSignal[4];
        signals[0] = drivePosition;
        signals[1] = driveVelocity;
        signals[2] = steerPosition;
        signals[3] = steerVelocity;

        lastAngle = getModuleState(true).angle;
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steeringMotor, swerveEncoder);
    }

    private void configureSwerveEncoder() {
        swerveEncoderConfigurator = swerveEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorDiscontinuityPoint = 1;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        swerveEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
        swerveEncoder.getAbsolutePosition().setUpdateFrequency(50);
    }

    private void configureSteeringMotor(SwerveModuleConstants moduleConstants) {
        steerMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = Constants.ModuleConstants.TalonFXModuleConstants.steerStatorCurrentLimitEnable;
        steerMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.ModuleConstants.TalonFXModuleConstants.steerStatorCurrentLimit;
        steerMotorConfiguration.Audio.BeepOnConfig = false;
        //steerMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        steerMotorConfiguration.Slot0.kP = Constants.ModuleConstants.TalonFXModuleConstants.steeringKp;
        steerMotorConfiguration.Slot0.kI = 0.0;
        steerMotorConfiguration.Slot0.kD = Constants.ModuleConstants.TalonFXModuleConstants.steeringKd;

        steerMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerMotorConfiguration.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.SteeringConversionFactor;
        steerMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steeringMotor.getConfigurator().apply(steerMotorConfiguration);
        steeringMotor.getPosition().setUpdateFrequency(50);
        resetToAbsolute();
    }

    private void configureDriveMotor(SwerveModuleConstants moduleConstants) {
        driveMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = Constants.ModuleConstants.TalonFXModuleConstants.driveStatorCurrentLimitEnable;
        driveMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.ModuleConstants.TalonFXModuleConstants.driveStatorCurrentLimit;
        driveMotorConfiguration.Audio.BeepOnConfig = false;

        driveMotorConfiguration.Slot0.kP = Constants.ModuleConstants.TalonFXModuleConstants.driveKp;
        driveMotorConfiguration.Slot0.kI = 0.0;
        driveMotorConfiguration.Slot0.kD = Constants.ModuleConstants.TalonFXModuleConstants.driveKd;

        driveMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotorConfiguration.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.DriveConversionPositionFactor;
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotor.getConfigurator().apply(driveMotorConfiguration);

        driveMotor.getConfigurator().setPosition(0.0);
        driveMotor.getVelocity().setUpdateFrequency(50);
    }

    /**
     * Returns the module number associated with this module.
     *
     * @return The module number.
     */

    public int getModuleNumber() {
        return moduleNumber;
    }

    public double getDriveMotor() {
        return driveMotor.get();
    }

    public void setDriveVoltage(Voltage voltage) {
        driveMotor.setVoltage(voltage.in(Volts));
    }

    /**
     * Sets the neutral mode of the steer motor.<p>
     * Use {@code}NeutralModeValue.Brake{@code} or {@code}NeutralModeValue.Coast{@code}
     * 
     * @param mode The NeutralModeValue to set.
     */

    public void setSteerNeutralMode(NeutralModeValue mode){
        steeringMotor.setNeutralMode(mode);
    }

    /**
     * Sets the neutral mode of the drive motor.<p>
     * Use {@code}NeutralModeValue.Brake{@code} or {@code}NeutralModeValue.Coast{@code}
     * 
     * @param mode The NeutralModeValue to set.
     */

    public void setDriveNeutralMode(NeutralModeValue mode){
        driveMotor.setNeutralMode(mode);
    }

    /**
     * Sets the desired state of the Swerve module, including the speed and angle.
     *
     * @param desiredState The desired state of the Swerve module.
     * @param isOpenLoop   Whether the desired state is open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */

    public void set(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = OnboardModuleState.optimize(desiredState, getAngle());
        setSpeed(desiredState, isOpenLoop);
        setAngle(desiredState);
    }

    /**
     * Sets the speed of the swerve module based on the desired state's speed and whether it is in open loop or closed loop control.
     *
     * @param desiredState The desired state of the swerve module.
     * @param isOpenLoop Whether to drive in an open loop (Tele-Op) or closed loop (Autonomous) state.
     */

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            driveMotor.setControl(driveDutyCyleRequest.withOutput(desiredState.speedMetersPerSecond / SwerveConstants.PhysicalMaxVelocity));
        } else {
            double velocity = Conversions.mpsToFalconRPS(desiredState.speedMetersPerSecond, SwerveConstants.WheelCircumference, 1.0);
            double feedforward = this.feedforward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocityRequest.withVelocity(velocity).withFeedForward(feedforward));
        }
    }

    /**
     * Sets the angle of the swerve module based on the desired state's angle.
     *
     * @param desiredState The desired state of the swerve module.
     */

    private void setAngle(SwerveModuleState desiredState) {
        /* Prevent rotating module if speed is less then 1%. Prevents jittering when not moving. */
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.PhysicalMaxVelocity * 0.01)) ? lastAngle : desiredState.angle;

        steeringMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
        lastAngle = angle;
    }

    /**
     * Retrieves the current agnle of the angle motor.
     *
     * @return The angle of the rotation motor as a Rotation2d.
     */

    private Rotation2d getAngle() {
        return internalState.angle;
    }

    /**
     * Retrieves the current absolute rotation of the CANcoder.
     *
     * @return The current absolute rotation of the CANcoder sensor as a Rotation2d.
     */

    public Rotation2d getEncoderAngle() {
        return Rotation2d.fromRotations(swerveEncoder.getAbsolutePosition().refresh().getValue().in(Rotations));
    }

    public void resetModulePosition(){
        driveMotor.setPosition(0.0);
    }

    public void resetToAbsolute() {
        double absolutePosition = swerveEncoder.getAbsolutePosition().waitForUpdate(0.5).getValue().in(Rotations);
        steeringMotor.setPosition(absolutePosition);
    }

    /**
     * Retrieves the current state of the swerve module.
     *
     * @param refresh Whether to refresh the readings from the motors.
     * 
     * @return The state of the swerve module, including the velocity (m/s) and angle.
     */

    public SwerveModuleState getModuleState(boolean refresh) {
        if(refresh) {
            driveVelocity.refresh();
            steerPosition.refresh();
        }

        double speedMetersPerSecond = Conversions.falconRPSToMechanismMPS(
            driveMotor.getVelocity().getValue().in(RotationsPerSecond), 
            SwerveConstants.WheelCircumference, 
            1.0);

        Rotation2d angle = Rotation2d.fromRotations(steerPosition.getValue().in(Rotations));

        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    /**
     * Retrieves the current position of the swerve module.
     *
     * @param refresh Whether to refresh the readings from the motors.
     * 
     * @return The position of the swerve module, consisting of the distance traveled in meters and the angle.
     */

    public SwerveModulePosition getModulePosition(boolean refresh) {
        if(refresh) {
            drivePosition.refresh();
            driveVelocity.refresh();
            steerPosition.refresh();
            steerVelocity.refresh();
        }
        
        double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity).in(Rotations);
        double steerRotations = BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity).in(Rotations);

        double distance = Conversions.falconRotationsToMechanismMeters(driveRotations, SwerveConstants.WheelCircumference, 1.0); // -0.4
        Rotation2d angle = Rotation2d.fromRotations(steerRotations);

        internalState.distanceMeters = distance;
        internalState.angle = angle;
        
        return internalState;
    }

    public BaseStatusSignal[] getSignals() {
        return signals;
    }

    public void stopAll(){
        driveMotor.stopMotor();
        steeringMotor.stopMotor();
    }
}