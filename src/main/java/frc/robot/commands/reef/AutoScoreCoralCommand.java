package frc.robot.commands.reef;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import frc.robot.subsystems.PathfindingSubsystem;
import frc.robot.subsystems.PathfindingSubsystem.FullRobotTargetState;
import frc.robot.utilities.MathUtilities;
import frc.robot.utilities.Reef;

public class AutoScoreCoralCommand extends Command {
	private final boolean right;
	private final int branchIndex;
	private final boolean canCancel;
	private SequentialCommandGroup commands;

	public AutoScoreCoralCommand(boolean right, int branchIndex, boolean canCancel) {
		this.right = right;
		this.branchIndex = branchIndex;
		this.canCancel = canCancel;

		addRequirements(RobotContainer.armSubsystem, RobotContainer.elevatorSubsystem, RobotContainer.swerveSubsystem, RobotContainer.intakeSubsystem);
	}

	@Override
	public void initialize() {
		this.commands = new SequentialCommandGroup();

		int closestTagID = Reef.getClosestReef(RobotContainer.swerveSubsystem.getPose());

		if (closestTagID != -1) {
			System.out.println("Scoring on level " + (branchIndex + 1));
			RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kBothRumble, 0.5, 0.05).schedule();

			Pose2d tagPose = Reef.getReefTagPose(closestTagID, true);

			if (branchIndex != 0) {
				

				Pose2d reefBranchPose = Reef.getBranchTopPose(tagPose, right);

				// Get the angle of the branch and its perpendicular angle
				double branchAngle = Reef.getBranchAngle(branchIndex);

				double scoringAngle = -MathUtilities.AngleUtilities.getPerpendicularAngle(branchAngle).in(Radians);

				Pose3d endEffectorPose = new Pose3d(reefBranchPose.getX(), 
													reefBranchPose.getY(),
													Units.feetToMeters(Reef.heightsList[branchIndex]),
													new Rotation3d(
														0, 
														scoringAngle,
														reefBranchPose.getRotation().getRadians()
														));

				// Calculates the minimum distance that the robot can be from the reef specificly from the shouler pivots POV
				double minimumRobotDistance = (Units.feetToMeters(Constants.RobotKinematicConstants.LENGTH) / 2) + Units.feetToMeters(Constants.ReefConstants.FieldConstants.BRANCH_FOWARD_OFFSET) + 0.01;
				// - Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD);

				FullRobotTargetState fullRobotTargetState = RobotContainer.pathfindingSubsystem
						.computeStateForEndEffectorPose(endEffectorPose, minimumRobotDistance);

				// The pose to go to that is a bit away so the arm and elevator have time to move into position
				Pose2d intermediatePose = fullRobotTargetState.chassisPose().plus(new Transform2d(-0.1, 0, Rotation2d.fromDegrees(0)));

				// The pose to go to when the robot is pulling away from the branch 
				Pose2d pullBackPose = fullRobotTargetState.chassisPose().plus(new Transform2d(-0.2, 0, Rotation2d.fromDegrees(0)));

				if (fullRobotTargetState.targetStatus() != PathfindingSubsystem.StateStatus.INVALID) {

					this.commands.addCommands(
							new MoveToPoseCommand(intermediatePose, false)
									.withDeadline(new ParallelCommandGroup(
											new SetElevatorHeightCommand(fullRobotTargetState.elevatorHeight()),
											new SetArmConfigurationCommand(
												fullRobotTargetState.shoulderAngle() + Constants.ReefConstants.LIFT_ANGLE, 90.0,
													true))),
							new ParallelCommandGroup(new MoveToPoseCommand(fullRobotTargetState.chassisPose(), true)),
							new ParallelCommandGroup(new SetArmConfigurationCommand(fullRobotTargetState.shoulderAngle(), 90.0, true)));

					if (branchIndex != 0) {
						this.commands.addCommands(new ParallelCommandGroup(RobotContainer.intakeSubsystem.intakeUntil(Constants.DriverConstants.OUTTAKE_SPEED, false, 3),
						new MoveToPoseCommand(pullBackPose, true).withTimeout(5)));
					} else {
						this.commands.addCommands(RobotContainer.intakeSubsystem.intakeUntil(Constants.DriverConstants.OUTTAKE_SPEED, false, 3));
					}
				} else {
					System.err.println("Nan value target detected in auto score");
					RobotContainer.driverController.setRumbleBlinkCommand(RumbleType.kBothRumble, 1, 0.15, 0.1, 2)
							.schedule();
				}
			
			} else {
				Pose2d newRobotPose = tagPose.plus(new Transform2d(
						-(Units.feetToMeters(Constants.RobotKinematicConstants.LENGTH) / 2)
								- Units.feetToMeters(Constants.ReefConstants.FieldConstants.L1.SCORE_OFFSET),
						0, Rotation2d.fromDegrees(0)));

				Pose2d intermediatePose = newRobotPose.plus(new Transform2d(-0.1, 0, Rotation2d.fromDegrees(0)));

				this.commands
						.addCommands(new MoveToPoseCommand(intermediatePose, false).withDeadline(
								new ParallelCommandGroup(
										new SetElevatorHeightCommand(Units
												.feetToMeters(Constants.ReefConstants.FieldConstants.L1.SCORE_HEIGHT)),
										new SetArmConfigurationCommand(
												Constants.ReefConstants.FieldConstants.L1.SCORE_ANGLE, 0.0, true))),
								new ParallelCommandGroup(
										new MoveToPoseCommand(newRobotPose, true).withTimeout(5)),
								new ParallelCommandGroup(RobotContainer.intakeSubsystem
										.intakeUntil(Constants.DriverConstants.OUTTAKE_SPEED, false, 3)));
			}

		} else {
			System.out.println("No close reef tag");
			RobotContainer.driverController.setRumbleBlinkCommand(RumbleType.kBothRumble, 1, 0.15, 0.1, 2).schedule();
		}

		this.commands.initialize();
	}

	@Override
	public void execute() {
		if (!RobotContainer.driverController.noDirectionsHeld() && canCancel) {
			this.cancel();
		}
		this.commands.execute();
	}

	@Override
	public boolean isFinished() {
		return this.commands.isFinished();
	}

	@Override
	public void end(boolean interrupted) {
		RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kBothRumble, 0.5, 0.05).schedule();
	}
}