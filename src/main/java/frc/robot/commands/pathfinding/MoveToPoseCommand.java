package frc.robot.commands.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MoveToPoseCommand extends Command {
    private Pose2d goalPose;
    private double goalEndSpeed;

    public MoveToPoseCommand(Pose2d goalPose, double goalEndSpeed) {
        this.goalPose = goalPose;
        this.goalEndSpeed = goalEndSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
