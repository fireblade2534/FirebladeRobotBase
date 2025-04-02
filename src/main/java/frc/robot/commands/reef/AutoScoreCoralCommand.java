package frc.robot.commands.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.utilities.Reef;

public class AutoScoreCoralCommand extends SequentialCommandGroup {
    public AutoScoreCoralCommand(boolean left) {
        int closestTagID = Reef.getClosestReef(RobotContainer.swerveSubsystem.getPose());

        if (closestTagID != -1) {
            Pose2d tagPose = Reef.getReefIndexPose(closestTagID, true);
        } else {
            // DO A CONTROLLER RUMBLE
        }
    }
}
