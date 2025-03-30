package frc.robot.commands.reef;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PathfindToPoseCommand;
public class AutoAlignWithReefCommandGroup extends SequentialCommandGroup {
    // The reef indexes start from 0 at the side of the reef farthest (and the left
    // reef pole) from the middle driverstation of your robots team. The indexes go
    // counter clockwise

    public AutoAlignWithReefCommandGroup(int reefIndex) {
        System.out.println("Generating auto align with reef index " + reefIndex);

        addCommands(new PathfindToPoseCommand(new Reef().getReefIndexPose(reefIndex).transformBy(Constants.ReefConstants.AlignConstants.alignOffset), 0), new PathfindToPoseCommand(new Reef().getReefIndexPose(reefIndex + 1).transformBy(Constants.ReefConstants.AlignConstants.alignOffset), 0));
    }
}
