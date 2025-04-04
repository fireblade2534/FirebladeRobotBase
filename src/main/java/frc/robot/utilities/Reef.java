package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Reef {
    public static double[] heightsList = {Constants.ReefConstants.FieldConstants.L1.MAX_HEIGHT, Constants.ReefConstants.FieldConstants.L2.MAX_HEIGHT, Constants.ReefConstants.FieldConstants.L3.MAX_HEIGHT, Constants.ReefConstants.FieldConstants.L4.MAX_HEIGHT};

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

    public static Pose2d getReefIDPose(int tagID, boolean facingTag) {
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
            Pose2d tagPose =  Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();

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
    public static Pose3d getBranchTopPose(Pose2d tagPose, boolean left) {
        Transform3d branchTransform = new Transform3d(Units.feetToMeters(Constants.ReefConstants.FieldConstants.BRANCH_FOWARD_OFFSET), Units.feetToMeters(Constants.ReefConstants.FieldConstants.BRANCH_LEFT_OFFSET), 0, new Rotation3d(0.0, 0.0, 0.0));
        return new Pose3d(new Translation3d(tagPose.getTranslation()), new Rotation3d(tagPose.getRotation())).transformBy(branchTransform);
    }



    /**
     * 
     * @param elevatorHeight the height of elevator in meters
     * @return
     */
    public static int getInferedBranchLevel(double elevatorHeight) {
        System.out.println(elevatorHeight);
        int closestIndex = -1;
        double closestDistance = Double.MAX_VALUE;

        for (int index = 0; index < heightsList.length; index++) {
            double heightDiff = Math.abs(heightsList[index] - Units.metersToFeet(elevatorHeight));
            if (heightDiff < closestDistance) {
                closestDistance = heightDiff;
                closestIndex = index;
            }
        }

        return closestIndex;
    }


}
