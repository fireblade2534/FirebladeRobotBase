package frc.robot.commands.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SetElevatorHeightCommand;
import frc.robot.commands.pathfinding.MoveToPoseCommand;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.utilities.MathUtilities;
import frc.robot.utilities.Reef;

public class AutoScoreCoralCommand extends Command {
    private final boolean left;
    private SequentialCommandGroup commands;

    public AutoScoreCoralCommand(boolean left) {
        this.left = left;
    }

    @Override
    public void initialize() {
        this.commands = new SequentialCommandGroup();

        int closestTagID = Reef.getClosestReef(RobotContainer.swerveSubsystem.getPose());

        if (closestTagID != -1) {
            Pose2d tagPose = Reef.getReefIDPose(closestTagID, true);

            Pose3d reefBranchPose = Reef.getBranchTopPose(tagPose, left);

            System.out.println("TEST" + closestTagID);
            SmartDashboard.putNumberArray("Test/Pose", SmartDashboardSubsystem.convertPose3dToNumbers(reefBranchPose));

            Rotation2d rotationTowards = MathUtilities.PoseUtilities.rotationToTarget(RobotContainer.swerveSubsystem.getPose().getTranslation(), reefBranchPose.getTranslation().toTranslation2d());

            Pose2d newRobotPose = new Pose2d(RobotContainer.swerveSubsystem.getPose().getTranslation(), rotationTowards);

            SmartDashboard.putNumberArray("Test/newRobotPose", SmartDashboardSubsystem.convertPose2dToNumbers(newRobotPose));

            int branchIndex = Reef.getInferedBranchLevel(RobotContainer.elevatorSubsystem.getStage1Height() + RobotContainer.elevatorSubsystem.getStage2Height());

            this.commands.addCommands(new ParallelCommandGroup(new MoveToPoseCommand(newRobotPose, 0.05, 2), new SetElevatorHeightCommand(Reef.heightsList[branchIndex])));
        } else {
            System.out.println("No close reef tag");
            // DO A CONTROLLER RUMBLE
        } 

        this.commands.initialize();
    }

    @Override
    public void execute() {
        this.commands.execute();
    }

    @Override
    public boolean isFinished() {
        return this.commands.isFinished();
    }
}
