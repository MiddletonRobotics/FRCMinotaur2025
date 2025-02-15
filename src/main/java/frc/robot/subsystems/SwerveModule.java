//Import all that jazz you need for swerve drive coding.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.SwerveModuleConstants;

//Sets up swerve drive class with encoders. This section can and should be added to.
public class SwerveModule extends SubsystemBase {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SwerveModuleState expectedState = new SwerveModuleState();

    private SparkMax driveMotor;
    private SparkMax steeringMotor;

    private SparkMaxConfig driveConfiguration, steeringConfiguration;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder steeringEncoder;
    private CANcoder swerveEncoder;
    private CANcoderConfigurator swerveEncoderConfigurator;

    private final SparkClosedLoopController drivePIDController;
    private final SparkClosedLoopController anglePIDController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ModuleConstants.driveKS, Constants.ModuleConstants.driveKV, Constants.ModuleConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        swerveEncoder = new CANcoder(moduleConstants.swerveEncoderID);
        configureSwerveEncoder();

        steeringMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        steeringEncoder = steeringMotor.getEncoder();
        anglePIDController = steeringMotor.getClosedLoopController();
        steeringConfiguration = new SparkMaxConfig();
        configureSteeringMotor(moduleConstants);

        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getClosedLoopController();
        driveConfiguration = new SparkMaxConfig();
        configureDriveMotor(moduleConstants);

        lastAngle = getActualModuleState().angle;
    }

    private void configureSwerveEncoder() {
        swerveEncoderConfigurator = swerveEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorDiscontinuityPoint = 1;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        swerveEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    private void configureSteeringMotor(SwerveModuleConstants moduleConstants) {
        steeringConfiguration
            .inverted(Constants.SwerveConstants.angleInvert)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ModuleConstants.angleContinuousCurrentLimit)
            .voltageCompensation(Constants.ModuleConstants.voltageCompensation);
        steeringConfiguration.encoder
            .positionConversionFactor(Constants.SwerveConstants.SteeringConversionFactor);
        steeringConfiguration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ModuleConstants.steeringKp, 0.0, Constants.ModuleConstants.steeringKd);

        steeringMotor.configure(steeringConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetToAbsolute();
    }

    private void configureDriveMotor(SwerveModuleConstants moduleConstants) {
        driveConfiguration
            .inverted(Constants.SwerveConstants.driveInvert)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ModuleConstants.driveContinuousCurrentLimit)
            .voltageCompensation(Constants.ModuleConstants.voltageCompensation);
        driveConfiguration.encoder
            .positionConversionFactor(Constants.SwerveConstants.DriveConversionPositionFactor)
            .velocityConversionFactor(Constants.SwerveConstants.DriveConversionVelocityFactor);
        driveConfiguration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ModuleConstants.driveKp, 0.0, Constants.ModuleConstants.driveKd);

        driveMotor.configure(driveConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveEncoder.setPosition(0.0);
    }

    public Rotation2d getSwerveEncoder() {
        return Rotation2d.fromDegrees(swerveEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public double getDriveMotor() {
        return driveMotor.get();
    }

    public Rotation2d getSteeringAngularPosition() {
        return Rotation2d.fromDegrees(steeringEncoder.getPosition());
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public SwerveModuleState getDesiredModuleState() {
        return expectedState;
    }

    public SwerveModuleState getActualModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getSteeringAngularPosition());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getSteeringAngularPosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = getSwerveEncoder().getDegrees();
        steeringEncoder.setPosition(absolutePosition);
    }

    public void setDriveVoltage(Voltage voltage) {
        driveMotor.setVoltage(voltage);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.006) {
            driveMotor.set(0);
            steeringMotor.set(0);

            if(desiredState.angle == lastAngle) {
                resetToAbsolute();
            }

            return;
        }

        desiredState = OnboardModuleState.optimize(desiredState, getActualModuleState().angle);
        this.expectedState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.PhysicalMaxVelocity;
            driveMotor.set(percentOutput);
        } else {
            drivePIDController.setReference(desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.PhysicalMaxAngularVelocity * 0.01)) ? lastAngle : desiredState.angle;
        anglePIDController.setReference(angle.getDegrees(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

        if(Robot.isSimulation()) {
            steeringEncoder.setPosition(angle.getDegrees());
        }

        lastAngle = angle;
    }

    public void stopDriveMotor() {
        driveMotor.set(0);
    }

    public void stopAngleMotor() {
        steeringMotor.set(0);
    }

    public void stop() {
        stopDriveMotor();
        stopAngleMotor();
    }
}