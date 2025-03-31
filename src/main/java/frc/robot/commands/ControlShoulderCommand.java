package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ControlShoulderCommand extends Command {
    private final double shoulderSpeed;

    public ControlShoulderCommand(double shoulderSpeed) {
        this.shoulderSpeed = shoulderSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setShoulderSetpoint(RobotContainer.armSubsystem.getShoulderSetpoint() + (shoulderSpeed / 50));
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
