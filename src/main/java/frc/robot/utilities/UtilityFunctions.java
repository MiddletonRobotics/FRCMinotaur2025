package frc.robot.utilities;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class UtilityFunctions {
    public static final double EPSILON = 1e-3;

	public static Rotation2d boundRotation(Rotation2d rotation){
		return new Rotation2d(MathUtil.angleModulus(rotation.getRadians()));
	}

	public static Rotation2d boundRotation0_360(Rotation2d rotation){
		rotation = boundRotation(rotation);
		if(rotation.getDegrees() < 0)return Rotation2d.fromDegrees(rotation.getDegrees() + 360.0);
		return rotation;
	}

	public static Rotation2d getTangent(Translation2d start, Translation2d end){
		Translation2d dist = end.minus(start);
		return new Rotation2d(dist.getX(), dist.getY());
	}

    public static boolean epsilonEquals(double a, double b, final double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

	public double getStallTorque(double stallTorque, double freeSpeed, double speed){
		return -stallTorque/freeSpeed * speed + stallTorque;
	}
	
	public static Twist2d chassisSpeedsToTwist2d(ChassisSpeeds chassisSpeeds){		
        return new Twist2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
	}

	public static boolean epsilonEquals(final Twist2d twist, final Twist2d other, double epsilon) {
        return UtilityFunctions.epsilonEquals(twist.dx, other.dx, epsilon) && UtilityFunctions.epsilonEquals(twist.dy, other.dy, epsilon) && UtilityFunctions.epsilonEquals(twist.dtheta, other.dtheta, epsilon);
    }

	public static boolean epsilonEquals(final Twist2d twist, final Twist2d other) {
        return epsilonEquals(twist, other, EPSILON);
    }

	public static Twist2d getPoseLog(Pose2d transform){
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        
        if (Math.abs(cos_minus_one) < EPSILON) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }

        final Translation2d translation_part = transform.getTranslation().rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
	}

    public static boolean isRedAlliance() {
        boolean isRed = false;

        if (DriverStation.getAlliance().isEmpty()) {
            return isRed;
        } else {
            isRed = DriverStation.getAlliance().get() == Alliance.Red;
        }
        return isRed;
    }

    public static boolean withinMargin(double margin, double a, double b) {
        if (a + margin >= b && a - margin <= b) {
            return true;
        }
        return false;
    }

    public static boolean isStopped(double velocity) {
        return withinMargin(0.01, velocity, 0);
    }

    public static boolean isStopped(double velocity, double minSpeed) {
        return withinMargin(minSpeed, velocity, 0);
    }
    
    public static Rotation2d calculateAngleToPoint(Translation2d current, Translation2d target){
        double targetX = target.getX() - current.getX();
        double targetY = target.getY() - current.getY();
        return Rotation2d.fromRadians(Math.atan2(targetY, targetX));
    }

    public static double calculateDistanceToTranslation(Translation2d current, Translation2d target){
        return current.getDistance(target);
    }

    public static DoubleSupplier calculateDistanceToTranslation(Supplier<Translation2d> current, Supplier<Translation2d> target){
        return () -> current.get().getDistance(target.get());
    }

    public static double average(double... nums){
        if(nums.length == 0) return 0.0;
        
        double sum = 0.0;
        for(double num : nums){
            sum += num;
        }
        return sum / nums.length;
    }

    public static boolean valueBetween(double value, double upper, double lower){
        return value < upper && value > lower;
    }

    public static Supplier<Command> continuousCondtional(Command onTrue, Command onFalse, BooleanSupplier condition){
        return condition.getAsBoolean() ? () -> onTrue : () -> onFalse;
    }

    public static void logBasicMotorOutputs(String path, TalonFX motor){
        Logger.recordOutput(path + "/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/TempCelcius", motor.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/ControlMode", motor.getControlMode().getValue().toString());
    }

    public static void logServoMotorOutputs(String path, TalonFX motor){
        Logger.recordOutput(path + "/Measured/Position", motor.getPosition().getValueAsDouble());
        Logger.recordOutput(path + "/Measured/Velocity", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput(path + "/Measured/Acceleration", motor.getAcceleration().getValueAsDouble());
    }
}