package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DealgeafierSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ShootBarge extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private DealgeafierSubsystem dealgeafierSubsystem;

    public ShootBarge(ElevatorSubsystem elevatorSubsystem, DealgeafierSubsystem dealgeafierSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.dealgeafierSubsystem = dealgeafierSubsystem;

        addRequirements(elevatorSubsystem, dealgeafierSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
