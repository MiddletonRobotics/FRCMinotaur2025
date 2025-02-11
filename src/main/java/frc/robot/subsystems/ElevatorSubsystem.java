package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.RelativeEncoder;
import frc.robot.utilities.constants.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;
    private final RelativeEncoder leftElevatorEncoder, rightElevatorEncoder;
    private final SparkClosedLoopController leftElevatorController;
    private final SparkClosedLoopController rightElevatorController;
    
    private SparkMaxConfig leftConfiguration, rightConfiguration;
    private final SimpleMotorFeedforward feedforward;
    private double targetVelocity = 0.0;
    private double lastPosition = 0.0;

    public ElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(Constants.ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        leftElevatorEncoder = leftElevatorMotor.getEncoder();
        leftElevatorController = leftElevatorMotor.getClosedLoopController();
        leftConfiguration = new SparkMaxConfig();
        configureLeftElevator();

        rightElevatorMotor = new SparkMax(Constants.ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
        rightElevatorEncoder = rightElevatorMotor.getEncoder();
        rightElevatorController = rightElevatorMotor.getClosedLoopController();
        rightConfiguration = new SparkMaxConfig();
        configureRightElevator();

        feedforward = new SimpleMotorFeedforward(
            Constants.ElevatorConstants.kS,
            Constants.ElevatorConstants.kV,
            Constants.ElevatorConstants.kG
        );

        lastPosition = getPositionInches();
    }

    private void configureLeftElevator() {
        leftConfiguration
            .inverted(Constants.SwerveConstants.angleInvert)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT)
            .voltageCompensation(Constants.ElevatorConstants.VOLTAGE_COMPENSATION);
        
        leftConfiguration.encoder
            .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR);
        
        leftConfiguration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);

        leftElevatorMotor.configure(leftConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftElevatorController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE);
    }

    private void configureRightElevator() {
        rightConfiguration
            .inverted(Constants.SwerveConstants.driveInvert)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT)
            .voltageCompensation(Constants.ElevatorConstants.VOLTAGE_COMPENSATION);
        
        rightConfiguration.encoder
            .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);
        
        rightConfiguration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);

        rightElevatorMotor.configure(rightConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position (inches)", getPositionInches());
        SmartDashboard.putNumber("Elevator Target Position", leftElevatorController.getSetpoint());
        SmartDashboard.putNumber("Elevator Target Velocity", targetVelocity);
        SmartDashboard.putBoolean("At Position", atSetpoint());
    }

    public void setPosition(double targetPosition) {
        targetPosition = MathUtil.clamp(
            targetPosition,
            Constants.ElevatorConstants.MIN_HEIGHT,
            Constants.ElevatorConstants.MAX_HEIGHT
        );

        if (Math.abs(targetPosition - getPositionInches()) < 0.001) {
            stop();
            return;
        }

        double positionError = targetPosition - getPositionInches();
        targetVelocity = Math.signum(positionError) * 
                        Math.min(Math.abs(positionError), Constants.ElevatorConstants.MAX_VELOCITY);

        leftElevatorController.setReference(targetPosition, SparkMax.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, feedforward.calculate(targetVelocity));
        rightElevatorController.setReference(targetPosition, SparkMax.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, feedforward.calculate(targetVelocity));

        lastPosition = targetPosition;
    }

    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -Constants.ElevatorConstants.MAX_SPEED, Constants.ElevatorConstants.MAX_SPEED);
        targetVelocity = speed * Constants.ElevatorConstants.MAX_VELOCITY;
        
        double ffOutput = feedforward.calculate(targetVelocity) / 12.0;
        leftElevatorMotor.set(speed + ffOutput);
        rightElevatorMotor.set(speed + ffOutput);
    }

    public double getPositionInches() {
        return leftElevatorEncoder.getPosition() * Constants.ElevatorConstants.INCHES_PER_ROTATION;
    }

    public double getVelocityInchesPerSecond() {
        return leftElevatorEncoder.getVelocity() * (Constants.ElevatorConstants.INCHES_PER_ROTATION / 60.0);
    }

    public boolean atSetpoint() {
        return leftElevatorController.setReference(360, SparkMax.ControlType.kPosition);
    }

    public void resetEncoders() {
        leftElevatorEncoder.setPosition(0);
        rightElevatorEncoder.setPosition(0);
    }

    public void stop() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
        
        targetVelocity = 0.0;
    }
}