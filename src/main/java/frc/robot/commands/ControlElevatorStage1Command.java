package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ControlElevatorStage1Command extends Command {
    private final double elevatorSpeed;

    public ControlElevatorStage1Command(double elevatorSpeed) {
        this.elevatorSpeed = elevatorSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.elevatorSubsystem.setStage1EncoderSetpoint(RobotContainer.elevatorSubsystem.getStage1EncoderSetpoint() + (elevatorSpeed / 50));
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
