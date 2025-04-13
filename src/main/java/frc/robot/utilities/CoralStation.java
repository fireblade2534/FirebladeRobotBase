package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class CoralStation {

    public static Pose2d getCoralStationTagPose(int tagID, boolean facingTag) {
        Pose2d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
        if (facingTag) {
            tagPose = tagPose.transformBy(new Transform2d(0, 0, new Rotation2d(Units.degreesToRadians(180))));
        }
        return tagPose;
    }

    /**
     * Returns the closest coral station tag to a given pose
     * 
     * @param pose
     * @return
     */
    public static int getClosestCoralStation(Pose2d pose) {
        int[] tagPoses = AllianceUtilities.isBlueAlliance()
                ? Constants.CoralStationConstants.FieldConstants.BLUE_ALLIANCE_CORAL_STATION_TAG_IDS
                : Constants.CoralStationConstants.FieldConstants.RED_ALLIANCE_CORAL_STATION_TAG_IDS;

        double lowestTagDistance = Double.MAX_VALUE;
        int lowestTagID = -1;

        for (int tagID : tagPoses) {
            Pose2d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();

            double tagDistance = tagPose.getTranslation().getDistance(pose.getTranslation());
            if (tagDistance < lowestTagDistance) {
                lowestTagDistance = tagDistance;
                lowestTagID = tagID;
            }
        }

        return lowestTagID;
    }

    /**
     * 
     * @param pose The pose of the tag
     * @return True if the tag is on the right side of the field, false if it is on the left side of the field
     */
    public static boolean getCoralStationSide(Pose2d pose) {
        return pose.getY() < (Constants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2);
    }

    public static Pose2d applyCoralStationOffsets(Pose2d tagPose, boolean right) {
        Transform2d coralStationTransform = new Transform2d(
                Units.feetToMeters(Constants.CoralStationConstants.FOWARD_OFFSET),
                Units.feetToMeters(Constants.CoralStationConstants.RIGHT_OFFSET) * ((right) ? -1 : 1),
                Rotation2d.fromDegrees(0));
        return tagPose.transformBy(coralStationTransform);
    }
}
