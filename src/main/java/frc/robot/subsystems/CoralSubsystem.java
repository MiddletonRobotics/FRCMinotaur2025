package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.BiFunction;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.utilities.ShuffleData;
import frc.robot.utilities.UtilityFunctions;



public class CoralSubsystem extends SubsystemBase {

    private SparkMax coralSpinnerMotor;

    private SparkMaxConfig pivotingConfiguration;
    private SparkMaxConfig CoralSpinnerConfiguration;

    //private RelativeEncoder algeaElevatorPivotingEncoder;
    private RelativeEncoder CoralSpinnerEncoder;

    //private SparkClosedLoopController pivotingPIDController;
    private SparkClosedLoopController CoralSpinnerPIDController;

    private DigitalInput firstBeamBreak;
    private DigitalInput secondBeamBreak;


    public CoralSubsystem() {
        coralSpinnerMotor = new SparkMax(21, MotorType.kBrushless);

        CoralSpinnerEncoder = coralSpinnerMotor.getEncoder();
        CoralSpinnerPIDController = coralSpinnerMotor.getClosedLoopController();
        CoralSpinnerConfiguration = new SparkMaxConfig();
        configureCoralSpinner();

        firstBeamBreak = new DigitalInput(1);
        secondBeamBreak = new DigitalInput(2);
    }
    
    public void configureCoralSpinner() {
        CoralSpinnerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(70);
            CoralSpinnerConfiguration.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            CoralSpinnerConfiguration.closedLoop
            .pid(0.01, 0.0, 0.0);

        coralSpinnerMotor.configure(CoralSpinnerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        CoralSpinnerEncoder.setPosition(0.0);
    }

    public void spinCoral(double speed) {
        coralSpinnerMotor.set(speed);
    }

    public void stopCoral() {
        coralSpinnerMotor.set(0);
    }

    public void slowCoral() {
        coralSpinnerMotor.set(0.1);
    }

    public boolean firstBeamBroken() {
        return !firstBeamBreak.get();
    }

    public boolean secondBeamBreak() {
        return !secondBeamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral First Beam Break", firstBeamBroken());
        SmartDashboard.putBoolean("Coral Second Beam Break", secondBeamBreak());

    }
}

