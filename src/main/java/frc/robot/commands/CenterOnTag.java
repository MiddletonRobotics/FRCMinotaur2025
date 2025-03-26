package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

// Used to get a general location, not exact
public class CenterOnTag extends Command {
    CommandSwerveDrivetrain dt;
    Limelight ll;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    // Note: xDis, xProportion, xVelocity, xError, and xOffset relate to ySpeed
    // Note: yDis, yProportion, yVelocity, yError, and yOffset relate to xSpeed
    private static GenericEntry rotProportion = Shuffleboard.getTab("Limelight").add("rotProportion", 2).getEntry();
    private static GenericEntry xProportion = Shuffleboard.getTab("Limelight").add("xProportion", 2).getEntry();
    private static GenericEntry yProportion = Shuffleboard.getTab("Limelight").add("yProportion", 2).getEntry();
    private static GenericEntry ySpeedEntry = Shuffleboard.getTab("Limelight").add("ySpeedEntry", 0.0).getEntry();
    private static GenericEntry xSpeedEntry = Shuffleboard.getTab("Limelight").add("xSpeedEntry", 0.0).getEntry();
    private static GenericEntry xDisEntry = Shuffleboard.getTab("Limelight").add("xDisEntry", 0.0).getEntry();
    private static GenericEntry yDisEntry = Shuffleboard.getTab("Limelight").add("yDisEntry", 0.0).getEntry();
    private static GenericEntry xErrorEntry = Shuffleboard.getTab("Limelight").add("xErrorEntry", 0.0).getEntry();
    private static GenericEntry yErrorEntry = Shuffleboard.getTab("Limelight").add("yErrorEntry", 0.0).getEntry();
    private static GenericEntry rotSpeedEntry = Shuffleboard.getTab("Limelight").add("rotSpeedEntry", 0.0).getEntry();
    private static GenericEntry yawAngleEntry = Shuffleboard.getTab("Limelight").add("yawAngleEntry", 0.0).getEntry();
    private static GenericEntry minXVelEntry = Shuffleboard.getTab("Limelight")
            .add("minXVelEntry", 0.1).getEntry();
    private static GenericEntry maxXVelEntry = Shuffleboard.getTab("Limelight")
            .add("maxXVelEntry", 1.0).getEntry();
    private static GenericEntry minYVelEntry = Shuffleboard.getTab("Limelight")
            .add("minYVelEntry", 0.1).getEntry();
    private static GenericEntry maxYVelEntry = Shuffleboard.getTab("Limelight")
            .add("maxYVelEntry", 1.0).getEntry();
    private static GenericEntry minAngVelEntry = Shuffleboard.getTab("Limelight")
            .add("minAngVelEntry", 0.0).getEntry();
    private static GenericEntry maxAngVelEntry = Shuffleboard.getTab("Limelight")
            .add("maxAngVelEntry", 20.0).getEntry();
    private static GenericEntry xOffsetEntry = Shuffleboard.getTab("Limelight")
            .add("xOffsetEntry", 0.0).getEntry();
    private static GenericEntry yOffsetEntry = Shuffleboard.getTab("Limelight")
            .add("yOffsetEntry", 0.7).getEntry();
    private static GenericEntry maxAccEntry = Shuffleboard.getTab("Limelight")
            .add("maxAccEntry", 3.0).getEntry();
    private static GenericEntry maxDccEntry = Shuffleboard.getTab("Limelight")
            .add("maxDccEntry", 8.0).getEntry();
    private static GenericEntry maxAngAccEntry = Shuffleboard.getTab("Limelight")
            .add("maxAngAccEntry", 8.0).getEntry();
    private static GenericEntry maxAngDccEntry = Shuffleboard.getTab("Limelight")
            .add("maxAngDccEntry", 16.0).getEntry();

    public CenterOnTag(CommandSwerveDrivetrain dt, Limelight ll) {

        this.dt = dt;
        this.ll = ll;
        addRequirements(dt);
    }

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double xError = 0;
    private double yError = 0;
    private double xOffset = 0;
    private double yOffset = 0;
    private double rotSpeed = 0;
    private double minXVel = 0;
    private double maxXVel = 0;
    private double minYVel = 0;
    private double maxYVel = 0;
    private double minAngVel = 0;
    private double maxAngVel = 0;
    private double maxAcc = 0;
    private double maxDcc = 0;
    private double maxAngAcc = 0;
    private double maxAngDcc = 0;
    private double commandedXVel = 0;
    private double commandedYVel = 0;
    private double commandedAngVel = 0;
    private double currentRotProportion = 0.0;
    private double currentXProportion = 0.0;
    private double currentYProportion = 0.0;
    private boolean llLost = false;

    @Override
    public void initialize() {
        // Set FieldRelative to false because we calculate based off of limelight camera
        minXVel = minXVelEntry.getDouble(0.1);
        maxXVel = maxXVelEntry.getDouble(1.0);
        minYVel = minYVelEntry.getDouble(0.1);
        maxYVel = maxYVelEntry.getDouble(1.0);
        xOffset = xOffsetEntry.getDouble(0.0);
        yOffset = yOffsetEntry.getDouble(0.7);
        minAngVel = minAngVelEntry.getDouble(0);
        maxAngVel = maxAngVelEntry.getDouble(10);
        maxAcc = maxAccEntry.getDouble(3.0) / (1 / 0.02);
        maxDcc = maxDccEntry.getDouble(8.0) / (1 / 0.02);
        maxAngAcc = maxAngAccEntry.getDouble(8.0) / (1 / 0.02);
        maxAngDcc = maxAngDccEntry.getDouble(16.0) / (1 / 0.02);
        currentRotProportion = rotProportion.getDouble(2);
        currentXProportion = xProportion.getDouble(2.0);
        currentYProportion = yProportion.getDouble(2.0);
        llLost = false;
        System.out.println("minVel=" + minXVel + " minAngVel=" + minAngVel + "  maxAngVel=" + maxAngVel);
    }

    @Override
    public void execute() {
        // Check if we have a valid target before calculating and setting values
        if (ll.getTimeSinceValid() == 0) {
            double rotAngleDegrees = -1 * ll.getFilteredYawDegrees();
            double xDis = -1 * ll.getxDistanceMeters();
            double yDis = ll.getzDistanceMeters();
            xError = xDis - xOffset;
            yError = yDis - yOffset;
            double calculatedXVel = Math.abs(xError * currentXProportion);
            double calculatedYVel = Math.abs(yError * currentYProportion);
            double calculatedAngVel = Math.abs(rotAngleDegrees * currentRotProportion);
            double desiredXVel;
            double desiredYVel;
            double desiredAngVel;
            double deltaVel;

            // Desired Velocities
            if (Math.abs(xError) < 0.03) {
                desiredXVel = 0;
            } else {
                desiredXVel = Math.copySign(MathUtil.clamp(calculatedXVel, minXVel, maxXVel), xError);
            }
            // if (Math.abs(yError) < LimelightConstants.yDisThreshold) {
            //     desiredYVel = 0;
            // } else {
            //     desiredYVel = Math.copySign(MathUtil.clamp(calculatedYVel, minYVel, maxYVel), yError);
            // }
            // if (Math.abs(ll.getYawAngleDegrees()) < LimelightConstants.rotThreshold) {
            //     desiredAngVel = 0;
            // } else {
            //     desiredAngVel = Math.copySign(MathUtil.clamp(calculatedAngVel, minAngVel, maxAngVel), rotAngleDegrees);
            // }

            // Commanded X Velocity ramped
            if ((Math.abs(desiredXVel) - Math.abs(commandedXVel))>0) {
                deltaVel = maxAcc;
            } else {
                deltaVel = maxDcc;
            }
            if (desiredXVel > commandedXVel) {
                commandedXVel = Math.min(desiredXVel, commandedXVel + deltaVel);
            } else if (desiredXVel < commandedXVel) {
                commandedXVel = Math.max(desiredXVel, commandedXVel - deltaVel);
            }

            // // Commanded Y Velocity ramped
            // if ((Math.abs(desiredYVel) - Math.abs(commandedYVel)) > 0) {
            //     deltaVel = maxAcc;
            // } else {
            //     deltaVel = maxDcc;
            // }
            // if (desiredYVel > commandedYVel) {
            //     commandedYVel = Math.min(desiredYVel, commandedYVel + deltaVel);
            // } else if (desiredYVel < commandedYVel) {
            //     commandedYVel = Math.max(desiredYVel, commandedYVel - deltaVel);
            // }

            // // Commanded angular Velocity ramped
            // if ((Math.abs(desiredAngVel) - Math.abs(commandedAngVel)) > 0) {
            //     deltaVel = maxAngAcc;
            // } else {
            //     deltaVel = maxAngDcc;
            // }
            // if (desiredAngVel > commandedAngVel) {
            //     commandedAngVel = Math.min(desiredAngVel, commandedAngVel + deltaVel);
            // } else if (desiredAngVel < commandedAngVel) {
            //     commandedAngVel = Math.max(desiredAngVel, commandedAngVel - deltaVel);
            // }

            ySpeed = commandedXVel;
            xSpeed = commandedYVel;
            rotSpeed = commandedAngVel;
            ySpeedEntry.setDouble(ySpeed);
            xSpeedEntry.setDouble(xSpeed);
            rotSpeedEntry.setDouble(rotSpeed);
            xDisEntry.setDouble(xDis);
            yDisEntry.setDouble(yDis);
            xErrorEntry.setDouble(xError);
            yErrorEntry.setDouble(yError);
            xOffsetEntry.setDouble(xOffset);
            yOffsetEntry.setDouble(yOffset);
            yawAngleEntry.setDouble(rotAngleDegrees);
            dt.setControl(
          drive.withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(Math.toRadians(rotSpeed)));
        } else {
            llLost = true;
        }

    }
}

//   @Override
//     public boolean isFinished() {
//         // if (Math.abs(xError) < LimelightConstants.xDisThreshold && Math
//         //         //.abs(yError) < LimelightConstants.yDisThreshold
//         //         && Math.abs(ll.getFilteredYawDegrees()) < LimelightConstants.rotThreshold) {
//         //     // Set FieldRelative back to true so the Drivetrain works for teleop
//         //     dt.setFieldRelative(true);
//         //     dt.drive(0, 0, 0);
//         //     System.out.println("COT command complete");
//         //     return true;
//         // } else if (llLost == true) {
//         //     System.out.println("COT command aborted because limelight lost");
//         //     // Set FieldRelative back to true so the Drivetrain works for teleop
//         //     dt.setFieldRelative(true);
//         //     dt.drive(0, 0, 0);
//         //     return true;
//         // }
//         // return false;
//     }
// }

