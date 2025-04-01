package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ControlWristCommand extends Command {
    private final double wristSpeed;

    public ControlWristCommand(double wristSpeed) {
        this.wristSpeed = wristSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.armSubsystem.setWristSetpoint(RobotContainer.armSubsystem.getWristSetpoint() + (wristSpeed / 50));
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
