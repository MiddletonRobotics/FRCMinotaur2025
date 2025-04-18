package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.constants.Constants;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private static boolean doRejectUpdate = false;
    public static String limelightUsed;
    private static LimelightHelper.PoseEstimate LLposeLeft;
    private static LimelightHelper.PoseEstimate LLposeRight;
    //Get average tag areas (percentage of image), Choose the limelight with the highest average tag area
    private static double limelightLeftAvgTagArea = 0;
    private static double limelightRightAvgTagArea = 0;

    private Field2d field;
    private static CommandSwerveDrivetrain system;
    private boolean waypointsTransformed = false;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private final static CommandSwerveDrivetrain instance;

    static {
        instance = TunerConstants.createDrivetrain();
    }

    public static CommandSwerveDrivetrain getInstance() {
        return instance;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        initializeOtf();
        configureAutoBuilder();

        field = new Field2d();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        initializeOtf();
        configureAutoBuilder();

        field = new Field2d();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        initializeOtf();
        configureAutoBuilder();

        field = new Field2d();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    
    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.m_sysIdRoutineToApply.dynamic(direction);
    }

     /**
     * <ul>
     * <li>Initializes the on-the-fly (OTF) waypoints for the robot
     * <li>Transforms the waypoints for the red alliance if needed
     * </ul>
     */
    public void initializeOtf() {
        if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) && !waypointsTransformed) {
            waypointsTransformed = true;

            transformWaypointsForAlliance(Constants.LimelightConstants.STATION_WAYPOINTS);
            transformWaypointsForAlliance(Constants.LimelightConstants.LEFT_REEF_WAYPOINTS);
            transformWaypointsForAlliance(Constants.LimelightConstants.RIGHT_REEF_WAYPOINTS);
        }
    }

    private void transformWaypointsForAlliance(List<Pose2d> waypoints) {
        final double FIELD_LENGTH = 17.55;
        final double X_OFFSET = 0.0;

        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d bluePose = waypoints.get(i);
            waypoints.set(
                i,
                new Pose2d(
                    FIELD_LENGTH - bluePose.getX() + X_OFFSET,
                    bluePose.getY(),
                    Rotation2d.fromDegrees(180 - bluePose.getRotation().getDegrees())
                )
            );
        }
    }

    /**
     * Uses PathPlanner's {@link AutoBuilder#pathfindToPose} to move to the desired pose
     *
     * @param pose The desired pose
     * @return A {@link DeferredCommand} that moves the robot to the desired pose
     */
    public Command goToPose(Pose2d pose) {
        return defer(
            () -> AutoBuilder.pathfindToPose(
                pose,
                new PathConstraints(
                    3.5,
                    4.0,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                )
            ).finallyDo((interrupted) -> this.stop())
        );
    }

    /**
     * Moves the robot to the nearest left or right reef
     *
     * @param reef Desired left or right reef from {@link Constants.Swerve.REEFS}
     * @return A {@link DeferredCommand} that moves the robot to the nearest left or right reef
     */
    public Command pathToReef(Constants.LimelightConstants.REEFS reef) {
        return defer(() -> {
            Pose2d target = null;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                target = this.getState().Pose.nearest(
                    (reef == Constants.LimelightConstants.REEFS.LEFT) ? Constants.LimelightConstants.RIGHT_REEF_WAYPOINTS : Constants.LimelightConstants.LEFT_REEF_WAYPOINTS
                );
            } else {
                target = this.getState().Pose.nearest(
                    (reef == Constants.LimelightConstants.REEFS.LEFT) ? Constants.LimelightConstants.LEFT_REEF_WAYPOINTS : Constants.LimelightConstants.RIGHT_REEF_WAYPOINTS
                );
            }

            return goToPose(target).withTimeout(0.01).andThen(goToPose(target));
        });
    }

    public Command flyToReef(Constants.LimelightConstants.REEFS reef) {
        Pose2d curPose = getState().Pose;
        Pose2d goalPose = null; 

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            goalPose = this.getState().Pose.nearest(
                (reef == Constants.LimelightConstants.REEFS.LEFT) ? Constants.LimelightConstants.LEFT_REEF_WAYPOINTS : Constants.LimelightConstants.RIGHT_REEF_WAYPOINTS
            );
        } else {
            goalPose = this.getState().Pose.nearest(
                (reef == Constants.LimelightConstants.REEFS.LEFT) ? Constants.LimelightConstants.LEFT_REEF_WAYPOINTS : Constants.LimelightConstants.RIGHT_REEF_WAYPOINTS
            );
        }
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(curPose.getX(), curPose.getY(), curPose.getRotation()),
            new Pose2d(goalPose.getX(), goalPose.getY(), Rotation2d.fromDegrees(0))
        );

        // The values are low so if anything goes wrong we can disable the robot
        PathConstraints constraints = new PathConstraints(2.5, 2.25, 2 * Math.PI, 4 * Math.PI);

        PathPlannerPath alignmentPath = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0, goalPose.getRotation())
        );

        SmartDashboard.putNumber("Goal Pose X", goalPose.getX());
        SmartDashboard.putNumber("Goal Pose Y", goalPose.getY());
        SmartDashboard.putNumber("Goal Pose Rotation", goalPose.getRotation().getDegrees());


        resetPose(getState().Pose);
        return AutoBuilder.followPath(alignmentPath);
    }

    public Command alignToRightReef (Constants.LimelightConstants.REEFS reef) {
        return new DeferredCommand(() -> flyToReef(reef), Set.of(this));
    }

    /**
     * Moves the robot to the red or blue processor depending on current alliance
     *
     * @return A {@link DeferredCommand} that moves the robot to the red or blue processor
     */
    public Command pathToProcessor() {
        return defer(() -> {
            Pose2d target = null;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                target = Constants.LimelightConstants.RED_PROCESSOR;
            } else {
                target = Constants.LimelightConstants.BLUE_PROCESSOR;
            }

            return goToPose(target).withTimeout(0.01).andThen(goToPose(target));
        });
    }

    /**
     * Moves the robot to the nearest coral station
     *
     * @return A {@link DeferredCommand} that moves the robot to the nearest coral station
     */
    public Command pathToStation() {
        return defer(() -> {
            Pose2d target = this.getState().Pose.nearest(Constants.LimelightConstants.STATION_WAYPOINTS);

            return goToPose(target).withTimeout(0.01).andThen(goToPose(target));
        });
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        /*
        * Periodically try to apply the operator perspective.
        * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        * This allows us to correct the perspective in case the robot code restarts mid-match.
        * Otherwise, only check and apply the operator perspective if the DS is disabled.
        * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        LimelightHelper.SetRobotOrientation("limelight-left", getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelper.SetRobotOrientation("limelight-right", getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LLposeLeft = get_LL_Estimate(false, "limelight-left");
        LLposeRight = get_LL_Estimate(false, "limelight-right");

        if (LLposeRight != null) {
            addVisionMeasurement(LLposeRight.pose, LLposeRight.timestampSeconds, VecBuilder.fill(0.7, 0.7, 0.3));
        } 
        
        if (LLposeLeft != null) {
            addVisionMeasurement(LLposeLeft.pose, LLposeLeft.timestampSeconds, VecBuilder.fill(0.7, 0.7, 0.3));
        }

        SmartDashboard.putNumber("Current Pose X", getState().Pose.getX());
        SmartDashboard.putNumber("Current Pose Y", getState().Pose.getY());
        SmartDashboard.putNumber("Current Pose Rotation", getState().Pose.getRotation().getDegrees());

        SmartDashboard.putNumber("Left Limelight Tag ID", LimelightHelper.getFiducialID("limelight-left"));
        SmartDashboard.putNumber("Right Limelight Tag ID", LimelightHelper.getFiducialID("limelight-right"));

        Pose2d currentPose = getState().Pose;
        field.setRobotPose(currentPose);

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData("Field",field);

        /* 

        List<Trajectory.State> trajectory_states = new ArrayList<Trajectory.State>();
        for (PathPlannerTrajectoryState state  : trajectory_states) {
            trajectory_states.add(new Trajectory.State(
                state.timeSeconds,
                state.linearVelocity,
                0,
                state.pose,
                0
            ));
        }

        */

        //Double[] fusedPose = {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians()};
        //SmartDashboard.putNumberArray("Fused PoseDBL", fusedPose);

        /*
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
    
                builder.addDoubleProperty("Front Left Angle", () -> getModule(0).getTargetState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> getModule(0).getTargetState().speedMetersPerSecond, null);
            
                builder.addDoubleProperty("Front Right Angle", () -> getModule(1).getTargetState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> getModule(1).getTargetState().speedMetersPerSecond, null);
            
                builder.addDoubleProperty("Back Left Angle", () -> getModule(2).getTargetState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> getModule(2).getTargetState().speedMetersPerSecond, null);
            
                builder.addDoubleProperty("Back Right Angle", () -> getModule(3).getTargetState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> getModule(3).getTargetState().speedMetersPerSecond, null);
            
                builder.addDoubleProperty("Robot Angle", () -> getState().Pose.getRotation().getRadians(), null);
            }
        });
        */
    }

    public void addMeasuremrent() {
    }

    private void updateOdometry() {
        LLposeLeft = get_LL_Estimate(false, "limelight-left");
        LLposeRight = get_LL_Estimate(false, "limelight-right");

        if (LLposeRight != null) {
            SmartDashboard.putNumber("RightLimelightPoseX", LLposeRight.pose.getY());
            SmartDashboard.putNumber("RightLimelightPoseY", LLposeRight.pose.getX());
            SmartDashboard.putNumber("RightLimelightPoseRot", LLposeRight.pose.getRotation().getDegrees());
            setStateStdDevs(VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE));
            addVisionMeasurement(LLposeRight.pose, LLposeRight.timestampSeconds);
        } 
        
        if (LLposeLeft != null) {
            SmartDashboard.putNumber("LeftLimelightPoseX", LLposeLeft.pose.getY());
            SmartDashboard.putNumber("LeftLimelightPoseY", LLposeLeft.pose.getX());
            SmartDashboard.putNumber("LeftLimelightPoseRot", LLposeLeft.pose.getRotation().getDegrees());
            setStateStdDevs(VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE));
            addVisionMeasurement(LLposeLeft.pose, LLposeLeft.timestampSeconds);
        }
    }

    public Command path_find_to(Pose2d pose, LinearVelocity endVelocity){
        return AutoBuilder.pathfindToPose(pose, TunerConstants.oTF_Constraints, endVelocity);
    }

    private LimelightHelper.PoseEstimate get_LL_Estimate(boolean useMegaTag2, String llName){
        doRejectUpdate = false;
        LimelightHelper.PoseEstimate poseEstimate = new LimelightHelper.PoseEstimate();

        if (useMegaTag2 == false) {
            poseEstimate = LimelightHelper.getBotPoseEstimate_wpiBlue(llName);

            if (poseEstimate == null) {
                doRejectUpdate = true;
            } else {
                if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
                    if (poseEstimate.rawFiducials[0].ambiguity > .7) {
                        doRejectUpdate = true;
                    }
                    if (poseEstimate.rawFiducials[0].distToCamera > 3) {
                        doRejectUpdate = true;
                    }
                
                }
                if (poseEstimate.tagCount == 0) {
                    doRejectUpdate = true;
                }
            }
        } else if (useMegaTag2 == true) {
            poseEstimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

            if (poseEstimate == null) {
                doRejectUpdate = true;
            } else {
                if (Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second,
                                                        // ignore vision updates. Might need to reduce to ~180
                {
                    doRejectUpdate = true;
                }
                if (poseEstimate.tagCount == 0) {
                    doRejectUpdate = true;
                }
            }
        }

        if (doRejectUpdate){
            return null;
        }
        else{
            return poseEstimate;
        }
    }

    public void choose_LL(){
        limelightLeftAvgTagArea = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("botpose").getDoubleArray(new double[11])[10];
        limelightRightAvgTagArea = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("botpose").getDoubleArray(new double[11])[10];
        SmartDashboard.putNumber("Left Limelight Tag Area", limelightLeftAvgTagArea);
        SmartDashboard.putNumber("Right Limelight Tag Area", limelightRightAvgTagArea);    

        if(limelightLeftAvgTagArea > 
        limelightRightAvgTagArea){
                limelightUsed = "limelight-left";
            }else{
                limelightUsed = "limelight-right";
        }
        
        SmartDashboard.putString("Limelight Used", limelightUsed);
    }

    private LimelightHelper.PoseEstimate get_manual_LL_Estimate(){
        choose_LL();
        LimelightHelper.PoseEstimate poseEstimate = new LimelightHelper.PoseEstimate();
        
        double[] botPose = LimelightHelper.getBotPose(limelightUsed);
        SmartDashboard.putNumberArray("Botpose", botPose);
        if (botPose.length != 0){
            if (botPose[0] == 0){
                return null;
            }
            poseEstimate.pose = new Pose2d(new Translation2d(botPose[0] + 8.7736 ,botPose[1] + 4.0257), new Rotation2d(Math.toRadians(botPose[5])));
        }

        Double[] pose = {poseEstimate.pose.getX(), poseEstimate.pose.getY(), poseEstimate.pose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("Manual Pose", pose);
        return poseEstimate;
    }

    public Command stop() {
        return runOnce(
            () -> this.setControl(
                new SwerveRequest.RobotCentric().withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
            )
        );
    }

    /**
     * Gets the {@link Swerve} subsystem instance
     *
     * @return The {@link Swerve} subsystem instance
     */
    public static CommandSwerveDrivetrain system() {
        if (system == null) {
            system = new CommandSwerveDrivetrain(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
            );
        }

        return system;
    }
}