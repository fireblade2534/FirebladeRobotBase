package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ControlElevatorStage2Command extends Command {
    private final double elevatorSpeed;

    public ControlElevatorStage2Command(double elevatorSpeed) {
        this.elevatorSpeed = elevatorSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.elevatorSubsystem.setStage2Setpoint(RobotContainer.elevatorSubsystem.getStage2Setpoint() + (elevatorSpeed / 50));
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
