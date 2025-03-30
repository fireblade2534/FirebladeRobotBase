package frc.robot.subsystems;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.AllianceUtilities;
import frc.robot.utilities.FileUtilities;
import frc.robot.utilities.JsonUtilities;

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
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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
}
