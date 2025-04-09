package frc.robot.commands.reef;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.pathfinding.MoveToPoseCommand;
import frc.robot.commands.pathfinding.PathfindToPoseCommand;
import frc.robot.utilities.Reef;
public class AutoAlignWithReefCommandGroup extends SequentialCommandGroup {
    // The reef indexes start from 0 at the side of the reef farthest (and the left
    // reef pole) from the middle driverstation of your robots team. The indexes go
    // counter clockwise

    public AutoAlignWithReefCommandGroup(int reefIndex) {
        System.out.println("Generating auto align with reef index " + reefIndex);
        Pose2d reefPose = Reef.getReefIndexPose(reefIndex, true);
        addCommands(new PathfindToPoseCommand(reefPose.transformBy(Constants.ReefConstants.AlignConstants.alignOffset.times(1.2)), 0), new MoveToPoseCommand(reefPose.transformBy(Constants.ReefConstants.AlignConstants.alignOffset), 0.05, 2, true));
    }
}
 