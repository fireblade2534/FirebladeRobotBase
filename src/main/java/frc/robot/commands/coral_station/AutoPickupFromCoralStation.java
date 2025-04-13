package frc.robot.commands.coral_station;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.commands.pathfinding.PathfindToPoseCommand;
import frc.robot.subsystems.PathfindingSubsystem.FullRobotTargetState;
import frc.robot.utilities.CoralStation;
import frc.robot.utilities.MathUtilities;
import frc.robot.utilities.Reef;

public class AutoPickupFromCoralStation extends Command {
    private final boolean canCancel;
    private SequentialCommandGroup commands;

    public AutoPickupFromCoralStation(boolean canCancel) {
        this.canCancel = canCancel;
    }

    @Override
    public void initialize() {
        this.commands = new SequentialCommandGroup();

        int closestTagID = CoralStation.getClosestCoralStation(RobotContainer.swerveSubsystem.getPose());

        if (closestTagID != -1) {
            System.out.println("Picking up coral from coral station");

            RobotContainer.driverController.setRumbleSecondsCommand(RumbleType.kBothRumble, 0.5, 0.05).schedule();

            Pose2d tagPose = CoralStation.getCoralStationTagPose(closestTagID, true);

            boolean right = CoralStation.getCoralStationSide(tagPose);

            Pose2d coralStationPose = CoralStation.applyCoralStationOffsets(tagPose, right);

            Pose3d endEffectorPose = new Pose3d(coralStationPose.getX(),
                    coralStationPose.getY(),
                    Units.feetToMeters(Constants.CoralStationConstants.VERTICAL_OFFSET),
                    new Rotation3d(
                            0,
                            Units.degreesToRadians(Constants.CoralStationConstants.PICK_UP_ANGLE),
                            coralStationPose.getRotation().getRadians()));

            // Calculates the minimum distance that the robot can be from the coral station specificly from the shouler pivots POV
            double minimumRobotDistance = (Units.feetToMeters(Constants.RobotKinematicConstants.LENGTH) / 2);

            FullRobotTargetState fullRobotTargetState = RobotContainer.pathfindingSubsystem
                    .computeStateForEndEffectorPose(endEffectorPose, minimumRobotDistance);

            double distanceToStation = RobotContainer.swerveSubsystem.getPose().getTranslation()
                    .getDistance(fullRobotTargetState.chassisPose().getTranslation());

            Translation2d intermediateTranslation = MathUtilities.PoseUtilities.interpolate2d(
                    fullRobotTargetState.chassisPose().getTranslation(),
                    RobotContainer.swerveSubsystem.getPose().getTranslation(),
                    Units.feetToMeters(Constants.CoralStationConstants.PATHFIND_DISTANCE) / distanceToStation);

            Pose2d intermediatePose = new Pose2d(intermediateTranslation, Rotation2d.fromRadians(endEffectorPose.getRotation().getZ()));

            this.commands.addCommands(new ParallelCommandGroup(new SetArmConfigurationCommand(fullRobotTargetState.shoulderAngle(), 0.0), new SetElevatorHeightCommand(fullRobotTargetState.elevatorHeight()))
                    .withDeadline(new PathfindToPoseCommand(intermediatePose, 0.1)),
                    new MoveToPoseCommand(fullRobotTargetState.chassisPose(), false).withDeadline(RobotContainer.intakeSubsystem.intakeUntil(Constants.DriverConstants.INTAKE_SPEED, true, 10)) );

        } else {
            System.out.println("No coral station tag found (This should not be possible)");
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
