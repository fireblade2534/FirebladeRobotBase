package frc.robot.commands.reef;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SetArmConfigurationCommand;
import frc.robot.commands.SetElevatorHeightCommand;
import frc.robot.commands.pathfinding.MoveToPoseCommand;
import frc.robot.utilities.MathUtilities;
import frc.robot.utilities.Reef;

public class AutoScoreCoralCommand extends Command {
    private final boolean right;
    private final int branchIndex;
    private SequentialCommandGroup commands;

    public AutoScoreCoralCommand(boolean right, int branchIndex) {
        this.right = right;
        this.branchIndex = branchIndex;

		addRequirements(RobotContainer.armSubsystem, RobotContainer.elevatorSubsystem, RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.commands = new SequentialCommandGroup();

        int closestTagID = Reef.getClosestReef(RobotContainer.swerveSubsystem.getPose());

        if (closestTagID != -1) {
            System.out.println("Scoring on level " + (branchIndex + 1));
	        RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kBothRumble, 0.5, 0.05).schedule();

            Pose2d tagPose = Reef.getReefIDPose(closestTagID, true);

            if (branchIndex != 0) {
                Pose2d reefBranchPose = Reef.getBranchTopPose(tagPose, right);

                // Get the angle of the branch and its perpendicular angle
                double branchAngle = Reef.getBranchAngle(branchIndex);
                double scoringAngle = -MathUtilities.AngleUtilities.getPerpendicularAngle(branchAngle).in(Degrees);

                // Get the branch translation
                Translation2d branchTranslation = reefBranchPose.getTranslation();

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

                double elevatorTargetHeight = Units.feetToMeters(Reef.heightsList[branchIndex])
                        - verticalRobotArmDistance;

                if (!RobotContainer.elevatorSubsystem.checkGlobalHeightPossible(elevatorTargetHeight)) {
                    System.out.println("Invalid elevator requirements for auto score trying backup requirements");

                    double branchPivotOffset = Units.feetToMeters(Reef.heightsList[branchIndex])
                            - RobotContainer.elevatorSubsystem.getPivotPointOffset(true);

                    horizontalRobotArmDistance = Math
                            .sqrt(Math.pow(Units.feetToMeters(Constants.ArmConstants.LENGTH), 2)
                                    - Math.pow(branchPivotOffset, 2))
                            + Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD);

                    scoringAngle = Units.radiansToDegrees(
                            Math.asin(branchPivotOffset / Units.feetToMeters(Constants.ArmConstants.LENGTH)));
                }

                double offsetDistance = -Math.max(horizontalRobotArmDistance,
                        (Units.feetToMeters(Constants.RobotKinematicConstants.LENGTH) / 2)
                                - Units.feetToMeters(Constants.ReefConstants.FieldConstants.BRANCH_FOWARD_OFFSET));

                Pose2d newRobotPose = new Pose2d(branchTranslation,
                        tagPose.getRotation()).plus(new Transform2d(offsetDistance, 0, Rotation2d.fromDegrees(0)));

                Pose2d pullBackPose = newRobotPose.plus(new Transform2d(-0.1, 0, Rotation2d.fromDegrees(0)));

                this.commands.addCommands(new ParallelCommandGroup( // MAKE THIS STUFF HAVE TIMEOUTS
                        new SetElevatorHeightCommand(elevatorTargetHeight),
                        new SetArmConfigurationCommand(scoringAngle + Constants.ReefConstants.LIFT_ANGLE, 90.0, true)),
                        new ParallelCommandGroup(new MoveToPoseCommand(newRobotPose, 0.08, 1).withTimeout(10) ),
                        new ParallelCommandGroup(new SetArmConfigurationCommand(scoringAngle, 90.0, true)),
                        new ParallelCommandGroup(RobotContainer.intakeSubsystem.intakeUntil(Constants.DriverConstants.OUTTAKE_SPEED, false, 3),new MoveToPoseCommand(pullBackPose, 0.08, 1)));
            } else {
                Pose2d newRobotPose = tagPose.plus(new Transform2d(
                        -(Units.feetToMeters(Constants.RobotKinematicConstants.LENGTH) / 2)
                                - Units.feetToMeters(Constants.ReefConstants.FieldConstants.L1.SCORE_OFFSET),
                        0, Rotation2d.fromDegrees(0)));

                this.commands
                        .addCommands(
                                new ParallelCommandGroup(
                                        new SetElevatorHeightCommand(Units
                                                .feetToMeters(Constants.ReefConstants.FieldConstants.L1.SCORE_HEIGHT)),
                                        new SetArmConfigurationCommand(
                                                Constants.ReefConstants.FieldConstants.L1.SCORE_ANGLE, 0.0, true)),
                                new ParallelCommandGroup(new MoveToPoseCommand(newRobotPose, 0.08, 1).withTimeout(10) ),
                                new ParallelCommandGroup(RobotContainer.intakeSubsystem.intakeUntil(Constants.DriverConstants.OUTTAKE_SPEED, false, 3))
                                );
            }

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
