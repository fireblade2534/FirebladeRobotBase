package frc.robot.commands.reef;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.utilities.AllianceUtilities;

public class Reef {
    public Pose2d getReefIndexPose(int reefIndex) {
        int tagID;
        System.out.println("IS BLUE " + AllianceUtilities.isBlueAlliance());
        if (AllianceUtilities.isBlueAlliance()) {
            tagID = Constants.ReefConstants.FieldConstants.BLUE_ALLIANCE_REEF_TAG_IDS[reefIndex];
        } else {
            tagID = Constants.ReefConstants.FieldConstants.RED_ALLIANCE_REEF_TAG_IDS[reefIndex];
        }
        return Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
    }
}
