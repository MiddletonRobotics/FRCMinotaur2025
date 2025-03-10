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

    private DigitalInput CoralLimitSwitch;

    public CoralSubsystem() {
        coralSpinnerMotor = new SparkMax(21, MotorType.kBrushless);

        CoralSpinnerEncoder = coralSpinnerMotor.getEncoder();
        CoralSpinnerPIDController = coralSpinnerMotor.getClosedLoopController();
        CoralSpinnerConfiguration = new SparkMaxConfig();
        configureCoralSpinner();
    }
    public void configureCoralSpinner() {
        CoralSpinnerConfiguration
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(45);
            CoralSpinnerConfiguration.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
            CoralSpinnerConfiguration.closedLoop
            .pid(0.01, 0.0, 0.0);

        coralSpinnerMotor.configure(CoralSpinnerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        CoralSpinnerEncoder.setPosition(0.0);
    }

    public void Coralspin(double speed) {
        coralSpinnerMotor.set(speed);
    }

    public void CoralSpinstop() {
        coralSpinnerMotor.set(0);
    }

    public boolean CoralfirstLimitBroken() {
        return CoralLimitSwitch.get();
    }

}

