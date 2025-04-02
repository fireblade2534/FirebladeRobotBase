package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Reef {
    public static Pose2d getReefIndexPose(int reefIndex, boolean facingTag) {
        int tagID;
        if (AllianceUtilities.isBlueAlliance()) {
            tagID = Constants.ReefConstants.FieldConstants.BLUE_ALLIANCE_REEF_TAG_IDS[reefIndex];
        } else {
            tagID = Constants.ReefConstants.FieldConstants.RED_ALLIANCE_REEF_TAG_IDS[reefIndex];
        }
        Pose2d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
        if (facingTag) {
            tagPose = tagPose.transformBy(new Transform2d(0,0, new Rotation2d(Units.degreesToRadians(180))));
        }
        return tagPose;
    }

    /**
     * Returns the closest tag to a given pose. If there is not any close tag it returns -1
     * 
     * @param pose
     * @return
     */
    public static int getClosestReef(Pose2d pose) {
        int[] tagPoses = AllianceUtilities.isBlueAlliance() ? Constants.ReefConstants.FieldConstants.BLUE_ALLIANCE_REEF_TAG_IDS : Constants.ReefConstants.FieldConstants.RED_ALLIANCE_REEF_TAG_IDS;
        double lowestTagDistance = Double.MAX_VALUE;
        int lowestTagID = -1;

        for (int tagID : tagPoses){
            Pose2d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();

            double tagDistance = tagPose.getTranslation().getDistance(pose.getTranslation());
            if (tagDistance < Constants.ReefConstants.CLOSE_DISTANCE && tagDistance < lowestTagDistance) {
                lowestTagDistance = tagDistance;
                lowestTagID = tagID;
            }
        }

        return lowestTagID;
    }

    /**
     * 
     * 
     * @param tagPose The pose of the tag when facing the tag
     * @param left
     */
    public static Pose2d getBranchTopPose(Pose2d tagPose, boolean left) {
        return new Pose2d();
    }
}
