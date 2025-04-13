package frc.robot.subsystems;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.AllianceUtilities;
import frc.robot.utilities.FileUtilities;
import frc.robot.utilities.JsonUtilities;
import frc.robot.utilities.MathUtilities;
import frc.robot.utilities.Reef;

public class PathfindingSubsystem extends SubsystemBase {
    public RobotConfig pathPlannerConfig;

    public PathfindingSubsystem() {
        createPathPlanner();
        System.out.println("Created PathfindingSubsystem");
    }

    private void createPathPlanner() {
        System.out.println("Syncing pathplanner config with constants");
        String pathPlannerFile = FileUtilities.readFile("pathplanner/settings.json");
        Map<String, Object> pathPlannerJson = JsonUtilities.fromJson(pathPlannerFile);

        List<Class<?>> swerveModules = List.of(Constants.SwerveConstants.SwerveModuleConstants.BackLeft.class,
                Constants.SwerveConstants.SwerveModuleConstants.BackRight.class,
                Constants.SwerveConstants.SwerveModuleConstants.FrontLeft.class,
                Constants.SwerveConstants.SwerveModuleConstants.FrontRight.class);

        String pathPlannerModified = JsonUtilities.pathPlannerToJson(pathPlannerJson, swerveModules);

        System.out.println("Saving synced pathplanner config");
        FileUtilities.writeFile("pathplanner/settings.json", pathPlannerModified);

        RobotContainer.configureNamedCommands();

        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> RobotContainer.swerveSubsystem.getPose(), // Robot pose supplier
                // Method to reset odometry (will be called if your auto has a starting pose)
                initialHolonomicPose -> RobotContainer.swerveSubsystem.resetOdometry(initialHolonomicPose),
                // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                () -> RobotContainer.swerveSubsystem.getRobotVelocity(),
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (Constants.SwerveConstants.ENABLE_FEED_FOWARD) {
                        RobotContainer.swerveSubsystem.swerveDrive.drive(
                                speedsRobotRelative,
                                RobotContainer.swerveSubsystem.swerveDrive.kinematics
                                        .toSwerveModuleStates(speedsRobotRelative),
                                moduleFeedForwards.linearForces());
                    } else {
                        RobotContainer.swerveSubsystem.setChassisSpeeds(speedsRobotRelative);
                    }
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                // optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                        // holonomic drive trains
                        new PIDConstants(Constants.SwerveConstants.TranslationPID.P,
                                Constants.SwerveConstants.TranslationPID.I, Constants.SwerveConstants.TranslationPID.D), // Translation PID constants
                        new PIDConstants(Constants.SwerveConstants.HeadingPID.P, Constants.SwerveConstants.HeadingPID.I,
                                Constants.SwerveConstants.HeadingPID.D) // Rotation PID constants
                ),
                pathPlannerConfig, // The robot configuration
                () -> !AllianceUtilities.isBlueAlliance(),
                RobotContainer.swerveSubsystem // Reference to this subsystem to set requirements
        );

        PathfindingCommand.warmupCommand().schedule();
    }

    public PathConstraints getPathConstraints() {
        return new PathConstraints(Units.feetToMeters(Constants.RobotKinematicConstants.MAX_SPEED),
                Units.feetToMeters(Constants.RobotKinematicConstants.MAX_ACCELERATION),
                Units.degreesToRadians(Constants.RobotKinematicConstants.MAX_ANGULAR_VELOCITY),
                Units.degreesToRadians(Constants.RobotKinematicConstants.MAX_ANGULAR_ACCELERATION));
    }

    public enum StateStatus {
        INVALID,
        CLOSE,
        PERFECT
    }

    public record FullRobotTargetState(Pose2d chassisPose,
            double elevatorHeight,
            double shoulderAngle,
            StateStatus targetStatus) {
    }

    public FullRobotTargetState computeStateForEndEffectorPose(Pose3d endEffectorPose, double minimumRobotDistance) {
        double scoringAngle = endEffectorPose.getRotation().getY();

        // Get the branch translation
        Translation2d branchTranslation = endEffectorPose.getTranslation().toTranslation2d();

        // Calculate the horizontal distance between the robot and the arm when at the
        // angle needed to score
        double horizontalRobotArmDistance = (Math.cos(scoringAngle)
                * Units.feetToMeters(Constants.ArmConstants.LENGTH))
                + Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD);

        // Calculate the verticle offset between the robot and the arm when at the
        // angle needed to score
        double verticalRobotArmDistance = Math.sin(scoringAngle)
                * Units.feetToMeters(Constants.ArmConstants.LENGTH);

        double elevatorTargetHeight = endEffectorPose.getZ()
                - verticalRobotArmDistance;

        StateStatus targetStatus = StateStatus.PERFECT;
        // For some locations the size of the robots swerve base and the limits on the elevators height prevents it from doing the optimal placing angles
        if (!RobotContainer.elevatorSubsystem.checkGlobalHeightPossible(elevatorTargetHeight)
                || minimumRobotDistance > horizontalRobotArmDistance) {
            targetStatus = StateStatus.CLOSE;

            double maxHeight = Units.feetToMeters(Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT)
                    + Units.feetToMeters(Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT);

            System.out.println("Invalid requirements for auto score trying backup requirements");

            double branchPivotOffset = endEffectorPose.getZ()
                    - RobotContainer.elevatorSubsystem.getPivotPointOffset(true);
            
            // If the branch is higher then the elevator can go the offset will be higher then max height
            // This code changes it so it is now the offset from the max travel of the elevator to the branch
            if (branchPivotOffset > maxHeight) {
                branchPivotOffset -= maxHeight;
            }

            // Calculates the horizontal distance between the shoulder pivot and the branch
            horizontalRobotArmDistance = Math
                    .sqrt(Math.pow(Units.feetToMeters(Constants.ArmConstants.LENGTH), 2)
                            - Math.pow(branchPivotOffset, 2))
                    + Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD);

            scoringAngle = Math.asin(branchPivotOffset / Units.feetToMeters(Constants.ArmConstants.LENGTH));
        }

        double offsetDistance = -Math.max(horizontalRobotArmDistance, minimumRobotDistance);

        // The pose the robot has to be in to score the coral
        Pose2d newRobotPose = new Pose2d(branchTranslation,
        Rotation2d.fromDegrees(endEffectorPose.getRotation().getZ())).plus(new Transform2d(offsetDistance, 0, Rotation2d.fromDegrees(0)));

        if (Double.isNaN(scoringAngle) || Double.isNaN(elevatorTargetHeight)) {
            targetStatus = StateStatus.INVALID;
        }

        return new FullRobotTargetState(newRobotPose, elevatorTargetHeight, Units.radiansToDegrees(scoringAngle), targetStatus);
    }
}
