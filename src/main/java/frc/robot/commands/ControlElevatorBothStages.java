package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ControlElevatorBothStages extends Command {
    private final Supplier<Double> elevatorSpeed;
    private final double elevatorOffset = RobotContainer.elevatorSubsystem.getPivotPointOffset();

    public ControlElevatorBothStages(Supplier<Double> elevatorSpeed) {
        this.elevatorSpeed = elevatorSpeed;

        addRequirements(RobotContainer.elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.elevatorSubsystem.setOverallHeight(RobotContainer.elevatorSubsystem.getStage1Setpoint() + RobotContainer.elevatorSubsystem.getStage2Setpoint() + ((elevatorSpeed.get() * Constants.DriverConstants.CONTROL_ELEVATOR_SPEED) / 50) + elevatorOffset);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
