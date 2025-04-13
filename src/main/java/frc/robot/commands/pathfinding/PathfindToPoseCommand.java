package frc.robot.commands.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PathfindToPoseCommand extends Command {
    private Command pathfindCommand;
    private Pose2d goalPose;
    private double goalEndSpeed;

    public PathfindToPoseCommand(Pose2d goalPose, double goalEndSpeed) {
        this.goalPose = goalPose;
        this.goalEndSpeed = goalEndSpeed;
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Pathfinding to pose: " + this.goalPose.toString());

        pathfindCommand = AutoBuilder.pathfindToPose(this.goalPose, RobotContainer.pathfindingSubsystem.getPathConstraints(), goalEndSpeed);
        RobotContainer.smartDashboardSubsystem.currentGoalPose = this.goalPose;
        pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        pathfindCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return pathfindCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pathfindCommand.end(interrupted);
    }
}
