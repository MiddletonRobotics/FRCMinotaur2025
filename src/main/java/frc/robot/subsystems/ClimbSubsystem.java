package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.AlgeaElevatorSubsystem.PivotingState;
import frc.robot.utilities.constants.Constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

    public class ClimbSubsystem extends SubsystemBase {



        private SparkMax climbMotor;
    
        private SparkMaxConfig climbConfiguration;
    
        private RelativeEncoder climbEncoder;
    
        //private SparkClosedLoopController pivotingPIDController;
        private SparkClosedLoopController ClimbPIDController;
    
       
    
    
        public ClimbSubsystem() {
        climbMotor = new SparkMax(22, MotorType.kBrushless);
    
        climbEncoder = climbMotor.getEncoder();
        ClimbPIDController = climbMotor.getClosedLoopController();
        climbConfiguration = new SparkMaxConfig();
            configureClimbMotor();
    
           
    
        }
        public void configureClimbMotor() {
            climbConfiguration
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(85);
                climbConfiguration.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
                climbConfiguration.closedLoop
                .pid(0.01, 0.0, 0.0);
    
                climbMotor.configure(climbConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                climbEncoder.setPosition(0.0);
        }
    
        public void ClimbMoveUp(double speed) {
            climbMotor.set(speed);
        }
    
        public void Climb() {
            climbMotor.set(0);
        }
    
        public void ClimbMoveDown(double speed) {
            climbMotor.set(speed);
        }
    
}
