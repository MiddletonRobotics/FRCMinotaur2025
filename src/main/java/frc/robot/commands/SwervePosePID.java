package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.LimelightHelper;

public class SwervePosePID extends Command {
    private PIDController xpidController = new PIDController(0.03, 0, 0);
    private PIDController ypidController = new PIDController(0.1, 0, 0);
  
    private CommandSwerveDrivetrain drivetrain;
    private Translation2d translation = new Translation2d();
    private boolean left;
    private Pose2d target;
    private double xVal, yVal;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    /** Creates a new SwervePosePID. */
    public SwervePosePID(CommandSwerveDrivetrain drivetrainGlobal, boolean isLeft) {
      this.drivetrain = drivetrainGlobal;
      this.left = isLeft;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      xpidController.setTolerance(0.1);
      ypidController.setTolerance(0.2);

      xVal = 0;
      yVal = 0;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(left){
          xVal = xpidController.calculate(LimelightHelper.getTX("limelight-left"), 0);
          yVal = -ypidController.calculate(LimelightHelper.getTY("limelight-left"), 0);
        }
        else{
            xVal = xpidController.calculate(LimelightHelper.getTX("limelight-right"), 0);
            yVal = -ypidController.calculate(LimelightHelper.getTY("limelight-right"), 0);
        }

        System.out.println("X Value " + xVal);
        System.out.println("Y Value " + yVal);

        translation = new Translation2d(yVal, xVal).times(1.5);
        drivetrain.setControl(
          drive.withVelocityX(translation.getX())
                .withVelocityY(translation.getY())
                .withRotationalRate(0)
        );        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if(Math.abs(xVal) <= 0.15 && Math.abs(yVal) <= 0.15){
        System.out.println("Finished PID");
        return true;
      }
      return false;
    }
  }
