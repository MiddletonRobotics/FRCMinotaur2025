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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Robot;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.SwerveModuleConstants;

//Sets up swerve drive class with encoders. This section can and should be added to.
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SwerveModuleState expectedState = new SwerveModuleState();

    private SparkMax driveMotor;
    private SparkMax angleMotor;

    private SparkMaxConfig driveConfig, angleConfig;

    private RelativeEncoder driveEncocder;
    private RelativeEncoder angleEncoder;
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

        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePIDController = angleMotor.getClosedLoopController();
        angleConfig = new SparkMaxConfig();
        configureAngleMotor(moduleConstants);

        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncocder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getClosedLoopController();
        driveConfig = new SparkMaxConfig();
        configureDriveMotor(moduleConstants);

        lastAngle = getSwerveModuleState().angle;
    }

    private void configureSwerveEncoder() {
        swerveEncoderConfigurator = swerveEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorDiscontinuityPoint = 1;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        swerveEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    private void configureAngleMotor(SwerveModuleConstants moduleConstants) {
        angleConfig
            .inverted(moduleConstants.angleInvert)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ModuleConstants.angleContinuousCurrentLimit)
            .voltageCompensation(Constants.ModuleConstants.voltageCompensation);
        angleConfig.encoder
            .positionConversionFactor(Constants.SwerveConstants.AngleConversionFactor);
        angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ModuleConstants.angleKP, Constants.ModuleConstants.angleKI, Constants.ModuleConstants.angleKD);

        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetToAbsolute();
    }

    private void configureDriveMotor(SwerveModuleConstants moduleConstants) {
        driveConfig
            .inverted(moduleConstants.driveInvert)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ModuleConstants.driveContinuousCurrentLimit)
            .voltageCompensation(Constants.ModuleConstants.voltageCompensation);
        driveConfig.encoder
            .positionConversionFactor(Constants.SwerveConstants.DriveConversionPositionFactor)
            .velocityConversionFactor(Constants.SwerveConstants.DriveConversionVelocityFactor);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ModuleConstants.driveKP, Constants.ModuleConstants.driveKI, Constants.ModuleConstants.driveKD);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveEncocder.setPosition(0.0);
    }

    public Rotation2d getSwerveEncoder() {
        return Rotation2d.fromDegrees(swerveEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public Rotation2d getDrivePosition() {
        return Rotation2d.fromDegrees(driveEncocder.getPosition());
    }

    public SwerveModuleState getDesiredState() {
        return expectedState;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(driveEncocder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(driveEncocder.getPosition(), getAngle());
    }

    public void resetToAbsolute() {
        double absolutePosition = getSwerveEncoder().getDegrees();
        angleEncoder.setPosition(absolutePosition);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.006) {
            driveMotor.set(0);
            angleMotor.set(0);

            if(desiredState.angle == lastAngle) {
                resetToAbsolute();
            }

            return;
        }

        desiredState = OnboardModuleState.optimize(desiredState, getSwerveModuleState().angle);
        this.expectedState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            drivePIDController.setReference(desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.PhysicalMaxSpeedMetersPerSecond * 0.01)) ? lastAngle : desiredState.angle;
        anglePIDController.setReference(angle.getDegrees(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

        if(Robot.isSimulation()) {
            angleEncoder.setPosition(angle.getDegrees());
        }

        lastAngle = angle;
    }

    public void stopDriveMotor() {
        driveMotor.set(0);
    }

    public void stopAngleMotor() {
        angleMotor.set(0);
    }

    public void stop() {
        stopDriveMotor();
        stopAngleMotor();
    }
}