package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ElevatorConstants;
import frc.robot.utilities.constants.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.Elevator.ElevatorMisc.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorMisc.ElevatorSparkMax;
import frc.robot.subsystems.Elevator.ElevatorMisc.OptixSpark;
import frc.robot.subsystems.Elevator.ElevatorMisc.ShuffleData;
import frc.robot.subsystems.Elevator.ElevatorMisc.ElevatorIO.ElevatorData;
import frc.robot.utilities.UtilityFunctions;



//import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
//import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
//import frc.robot.subsystems.elevator.real.ElevatorSparkMax;
//import frc.robot.subsystems.elevator.sim.ElevatorSimulation;
//import frc.robot.utils.ShuffleData;
//import frc.robot.utils.UtilityFunctions;



public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO elevatorio;
    private ElevatorData data = new ElevatorData();
    private ElevatorStates state = ElevatorStates.STOP;

    private ProfiledPIDController profile = new ProfiledPIDController(
            0,
            0,
            0,
            new TrapezoidProfile.Constraints(ElevatorConstants.ElevatorControl.maxV,
                    ElevatorConstants.ElevatorControl.maxA));

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(
            ElevatorConstants.ElevatorControl.kS,
            ElevatorConstants.ElevatorControl.kG,
            ElevatorConstants.ElevatorControl.kV,
            ElevatorConstants.ElevatorControl.kA);

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    private ShuffleData<Double> positionMetersLog = new ShuffleData<Double>("Elevator", "position", 0.0);
    private ShuffleData<Double> velocityMetersPerSecLog = new ShuffleData<Double>("Elevator", "velocity", 0.0);
    private ShuffleData<Double> accelerationMetersPerSecSquaredLog = new ShuffleData<Double>("Elevator", "acceleration",
            0.0);

    private ShuffleData<Double> leftAppliedVoltsLog = new ShuffleData<Double>("Elevator", "left applied volts", 0.0);
    private ShuffleData<Double> rightAppliedVoltsLog = new ShuffleData<Double>("Elevator", "right applied volts", 0.0);
    private ShuffleData<Double> leftCurrentAmpsLog = new ShuffleData<Double>("Elevator", "left current amps", 0.0);
    private ShuffleData<Double> rightCurrentAmpsLog = new ShuffleData<Double>("Elevator", "right current amps", 0.0);
    private ShuffleData<Double> leftTempCelciusLog = new ShuffleData<Double>("Elevator", "left temp celcius", 0.0);
    private ShuffleData<Double> rightTempCelciusLog = new ShuffleData<Double>("Elevator", "right temp celcius", 0.0);;

    private Mechanism2d mech = new Mechanism2d(3, 3);
    private MechanismRoot2d root = mech.getRoot("elevator", 1, 0);
    private MechanismLigament2d elevatorMech = root
            .append(new MechanismLigament2d("elevator", ElevatorConstants.ElevatorSpecs.baseHeight, 90));

    private double elevatorInnerStagePos;
    private double elevatorMiddleStagePos;

    StructPublisher<Pose3d> elevatorInnerStage = NetworkTableInstance.getDefault()
            .getStructTopic("Elevator Inner Stage", Pose3d.struct).publish();
    StructPublisher<Pose3d> elevatorMiddleStage = NetworkTableInstance.getDefault()
            .getStructTopic("Elevator Middle Stage", Pose3d.struct).publish();

    public Elevator() {
        if (Robot.isSimulation()) {
            elevatorio = new ElevatorSimulation();
        } else {
            elevatorio = new ElevatorSparkMax();
        }
    }

    public ElevatorStates getState() {
        return state;
    }

    public double getPositionMeters() {
        return data.positionMeters;
    }

    public double getVelocityRadPerSec() {
        return data.velocityMetersPerSecond;
    }

    /** returns true when the state is reached */
    public boolean getIsStableState() {
        switch (state) {
            case L1:
                return UtilityFunctions.withinMargin(0.01, ElevatorConstants.StateHeights.l1Height,
                        data.positionMeters);
            case L2:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.StateHeights.l2Height);
            case L3:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.StateHeights.l3Height);
            case L4:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.StateHeights.l4Height);
            case MAX:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.ElevatorSpecs.maxHeightMeters);
            case STOW:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.ElevatorSpecs.baseHeight);
            default:
                return false;
        }
    }

    public void setVoltage(double volts) {
        elevatorio.setVoltage(volts);
    }

    public void setState(ElevatorStates state) {
        this.state = state;
        switch (state) {
            case STOP:
                stop();
                break;
            case L1:
                setGoal(ElevatorConstants.StateHeights.l1Height);
                break;
            case L2:
                setGoal(ElevatorConstants.StateHeights.l2Height);
                break;
            case L3:
                setGoal(ElevatorConstants.StateHeights.l3Height);
                break;
            case L4:
                setGoal(ElevatorConstants.StateHeights.l4Height);
                break;
            case MAX:
                setGoal(ElevatorConstants.ElevatorSpecs.maxHeightMeters);
                break;
            case STOW:
                setGoal(ElevatorConstants.ElevatorSpecs.baseHeight);
                break;
            default:
                setGoal(0);
                break;
        }
    }

    public void setGoal(double height) {
        profile.setGoal(height);
    }

    private void runState() {
        switch (state) {
            case STOP:
                stop();
                break;
            default:
                moveToGoal();
                break;
        }
    }

    private void moveToGoal() {
        State firstState = profile.getSetpoint();
        profile.calculate(getPositionMeters());

        State nextState = profile.getSetpoint();
        double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

        elevatorio.setPosition(firstState.position, ffVoltage);
    }

    public void stop() {
        elevatorio.setVoltage(0);
    }

    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionMetersLog.set(data.positionMeters);
        velocityMetersPerSecLog.set(data.velocityMetersPerSecond);
        accelerationMetersPerSecSquaredLog.set(data.accelerationMetersPerSecondSquared);

        leftAppliedVoltsLog.set(data.leftAppliedVolts);
        rightAppliedVoltsLog.set(data.rightAppliedVolts);
        leftCurrentAmpsLog.set(data.leftCurrentAmps);
        rightCurrentAmpsLog.set(data.rightCurrentAmps);
        leftTempCelciusLog.set(data.leftTempCelcius);
        rightTempCelciusLog.set(data.rightTempCelcius);

        elevatorMech.setLength(ElevatorConstants.ElevatorSpecs.baseHeight + data.positionMeters);
        SmartDashboard.putData("elevator mechanism", mech);

        elevatorInnerStagePos = data.positionMeters / 2;
        elevatorMiddleStagePos = data.positionMeters - Units.inchesToMeters(1);
        elevatorInnerStage.set(new Pose3d(getTransform3d(elevatorInnerStagePos).getTranslation(),
                getTransform3d(elevatorInnerStagePos).getRotation()));
        elevatorMiddleStage.set(new Pose3d(getTransform3d(elevatorMiddleStagePos).getTranslation(),
                getTransform3d(elevatorMiddleStagePos).getRotation()));
    }

    private Transform3d getTransform3d(double pos) {
        Transform3d transform = new Transform3d(0, 0, pos, new Rotation3d(Angle.ofBaseUnits(0, Radians),
                Angle.ofBaseUnits(0, Radians), Angle.ofBaseUnits(0, Radians)));
        return transform;
    }

    @Override
    public void periodic() {
        elevatorio.updateData(data);
        runState();
        logData();
    }
}