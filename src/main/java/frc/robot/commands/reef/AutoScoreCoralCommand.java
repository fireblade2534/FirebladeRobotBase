package frc.robot.commands.reef;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SetArmConfigurationCommand;
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

            int branchIndex = Reef.getInferedBranchLevel(RobotContainer.elevatorSubsystem.getCarpetElevatorHeight());
            System.out.println("Infering branch level " + (branchIndex + 1));

            double branchAngle = Reef.getBranchAngle(branchIndex); // OK SO SCRAP THIS CODE AND FIGUREOUT HOW TO
                                                                   // CALCULATE THE PERPINDICULAR ANGLE THEN USING TRIG
                                                                   // CACULATE THE HIEHGT OF THE ARM

            double scoringAngle = -MathUtilities.AngleUtilities.getPerpendicularAngle(branchAngle).in(Degrees);

            // Get the swerve translation and the branch translation
            Translation2d swerveTranslation = RobotContainer.swerveSubsystem.getPose().getTranslation();
            Translation2d branchTranslation = reefBranchPose.getTranslation().toTranslation2d();

            // Get the required rotation to face the branch
            // Rotation2d rotationTowards = MathUtilities.PoseUtilities.rotationToTarget(
            // swerveTranslation,
            // branchTranslation);

            // Calculate the horizontal distance between the robot and the arm when at the
            // angle needed to score
            double horizontalRobotArmDistance = (Math.cos(Units.degreesToRadians(scoringAngle))
                    * Units.feetToMeters(Constants.ArmConstants.LENGTH))
                    + Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD);

            // Calculate the verticle offset between the robot and the arm when at the
            // angle needed to score
            double verticalRobotArmDistance = Math.sin(Units.degreesToRadians(scoringAngle))
            * Units.feetToMeters(Constants.ArmConstants.LENGTH);

            double offsetDistance = -Math.max(horizontalRobotArmDistance,
                    (Units.feetToMeters(Constants.RobotKinematicConstants.LENGTH) / 2)
                            - Units.feetToMeters(Constants.ReefConstants.FieldConstants.BRANCH_FOWARD_OFFSET));

            Pose2d newRobotPose = new Pose2d(branchTranslation,
                    tagPose.getRotation()).plus(new Transform2d(offsetDistance, 0, Rotation2d.fromDegrees(0)));

            Pose2d pullBackPose = newRobotPose.plus(new Transform2d(-0.1, 0, Rotation2d.fromDegrees(0)));

            this.commands.addCommands(new ParallelCommandGroup( // MAKE THIS STUFF HAVE TIMEOUTS
                    new SetElevatorHeightCommand(Units.feetToMeters(Reef.heightsList[branchIndex])  - verticalRobotArmDistance),
                    new SetArmConfigurationCommand(scoringAngle + 20, 90.0)
            ),
                    new ParallelCommandGroup(new MoveToPoseCommand(newRobotPose, 0.05, 1)),
                    new ParallelCommandGroup(new SetArmConfigurationCommand(scoringAngle, 90.0, true)),
                    new ParallelCommandGroup(new MoveToPoseCommand(pullBackPose, 0.05, 1)));
        } else {
            System.out.println("No close reef tag");
            RobotContainer.driverController.setRumbleBlinkCommand(RumbleType.kBothRumble, 1, 0.15, 0.1, 2).schedule();
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
