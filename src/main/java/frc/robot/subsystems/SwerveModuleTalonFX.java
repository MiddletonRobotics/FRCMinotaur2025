//Import all that jazz you need for swerve drive coding.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.SwerveModuleConstants;

//Sets up swerve drive class with encoders. This section can and should be added to.
public class SwerveModuleTalonFX extends SubsystemBase {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SwerveModuleState expectedState = new SwerveModuleState();

    private TalonFX driveMotor;
    private TalonFX steeringMotor;
    private CANcoder swerveEncoder;

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage steerPosition = new PositionVoltage(0);


    private CANcoderConfigurator swerveEncoderConfigurator;
    private TalonFXConfiguration driveMotorConfiguration, steerMotorConfiguration;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ModuleConstants.driveKS, Constants.ModuleConstants.driveKV, Constants.ModuleConstants.driveKA);

    public SwerveModuleTalonFX(int moduleNumber, SwerveModuleConstants moduleConstants) {
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

        lastAngle = getActualModuleState().angle;
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
        steerMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
        steerMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        steerMotorConfiguration.CurrentLimits.StatorCurrentLimit = 80;

        steerMotorConfiguration.Slot0.kP = Constants.ModuleConstants.steeringKp;
        steerMotorConfiguration.Slot0.kI = 0.0;
        steerMotorConfiguration.Slot0.kD = Constants.ModuleConstants.steeringKd;

        steerMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steerMotorConfiguration.Feedback.RotorToSensorRatio = Constants.SwerveConstants.SteeringConversionFactor;
        steerMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steeringMotor.getConfigurator().apply(steerMotorConfiguration);

    }

    private void configureDriveMotor(SwerveModuleConstants moduleConstants) {
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        driveMotorConfiguration.CurrentLimits.StatorCurrentLimit = 110;

        driveMotorConfiguration.Slot0.kP = Constants.ModuleConstants.driveKp;
        driveMotorConfiguration.Slot0.kI = 0.0;
        driveMotorConfiguration.Slot0.kD = Constants.ModuleConstants.driveKd;

        driveMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotorConfiguration.Feedback.RotorToSensorRatio = Constants.SwerveConstants.DriveConversionPositionFactor;
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotor.getConfigurator().apply(driveMotorConfiguration);

        driveMotor.setPosition(0.0);
        driveMotor.getVelocity().setUpdateFrequency(50);
    }

    public Rotation2d getSwerveEncoder() {
        return Rotation2d.fromDegrees(swerveEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public SwerveModuleState getDesiredModuleState() {
        return expectedState;
    }

    public SwerveModuleState getActualModuleState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.WheelCircumference), 
            Rotation2d.fromRotations(steeringMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), Constants.SwerveConstants.WheelCircumference), 
            Rotation2d.fromRotations(steeringMotor.getPosition().getValueAsDouble())
        );
    }

    public void resetToAbsolute() {
        double absolutePosition = swerveEncoder.getAbsolutePosition().waitForUpdate(0.5).getValueAsDouble();
        steeringMotor.setPosition(absolutePosition);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.006) {
            driveMotor.set(0);

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
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.PhysicalMaxVelocity;
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.WheelCircumference);
            driveVelocity.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.PhysicalMaxAngularVelocity * 0.01)) ? lastAngle : desiredState.angle;
        steeringMotor.setControl(steerPosition.withPosition(angle.getRotations()));

        if(Robot.isSimulation()) {
            steeringMotor.setPosition(angle.getDegrees());
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