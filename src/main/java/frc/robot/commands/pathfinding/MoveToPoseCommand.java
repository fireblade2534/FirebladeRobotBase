package frc.robot.commands.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.MathUtilities;

public class MoveToPoseCommand extends Command {
    private Pose2d goalPose;

    private final PIDController translationXController;
    private final PIDController translationYController;
    private final PIDController headingController;

    private final boolean endAtPose;

	private final double maxVelocity = Units.feetToMeters(Constants.SwerveConstants.MAX_POSE_MOVE_VELOCITY);

    /**
     * 
     * @param goalPose The pose to move to
     * @param endAtPose If the command should end once the robot has reached the goal pose
     */
    public MoveToPoseCommand(Pose2d goalPose, boolean endAtPose) {
        this.goalPose = goalPose;

        ///TrapezoidProfile.Constraints translatioConstraints = new TrapezoidProfile.Constraints(Units.feetToMeters(Constants.RobotKinematicConstants.MAX_SPEED), Units.feetToMeters(Constants.RobotKinematicConstants.MAX_ACCELERATION));

        this.translationXController = new PIDController(Constants.SwerveConstants.TranslationPID.P,
                Constants.SwerveConstants.TranslationPID.I, Constants.SwerveConstants.TranslationPID.D);
        this.translationYController = new PIDController(Constants.SwerveConstants.TranslationPID.P,
                Constants.SwerveConstants.TranslationPID.I, Constants.SwerveConstants.TranslationPID.D);
        this.headingController = new PIDController(Constants.SwerveConstants.HeadingPID.P,
                Constants.SwerveConstants.HeadingPID.I, Constants.SwerveConstants.HeadingPID.D);

				
        this.endAtPose = endAtPose;

        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

        this.translationXController.reset();
        this.translationYController.reset();
        this.headingController.reset();

        this.translationXController.setSetpoint(this.goalPose.getX());
        this.translationYController.setSetpoint(this.goalPose.getY());
        this.headingController.setSetpoint(this.goalPose.getRotation().getRadians());

        this.translationXController
                .setTolerance(Units.feetToMeters(Constants.SwerveConstants.TRANSLATION_ACCEPTABLE_ERROR));
        this.translationYController
                .setTolerance(Units.feetToMeters(Constants.SwerveConstants.TRANSLATION_ACCEPTABLE_ERROR));
        this.headingController
                .setTolerance(Units.degreesToRadians(Constants.SwerveConstants.ROTATION_ACCEPTABLE_ERROR));

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

		RobotContainer.smartDashboardSubsystem.currentGoalPose = this.goalPose;
        System.out.println("Moving to pose: " + this.goalPose.toString());
    }

	public void setNewPose(Pose2d newPose) {
		this.goalPose = newPose;
		this.translationXController.setSetpoint(this.goalPose.getX());
        this.translationYController.setSetpoint(this.goalPose.getY());
		this.headingController.setSetpoint(this.goalPose.getRotation().getRadians());
		RobotContainer.smartDashboardSubsystem.currentGoalPose = this.goalPose;
	}

    @Override
    public void execute() {
        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();
        double translationX = MathUtil.clamp(this.translationXController.calculate(robotPose.getX()), -maxVelocity, maxVelocity);
        double translationY = MathUtil.clamp(this.translationYController.calculate(robotPose.getY()), -maxVelocity, maxVelocity);

        double heading = this.headingController
                .calculate(RobotContainer.swerveSubsystem.swerveDrive.getOdometryHeading().getRadians());

        RobotContainer.swerveSubsystem.swerveDrive
                .driveFieldOriented(new ChassisSpeeds(translationX, translationY, heading / 1.5));
    }

    @Override
    public boolean isFinished() {
        if (!endAtPose) {
            return false;
        }

        return this.translationXController.atSetpoint()
                && this.translationYController.atSetpoint()
                && this.headingController.atSetpoint()
                && Math.abs(RobotContainer.swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond) < Units
                        .degreesToRadians(Constants.SwerveConstants.ROTATION_ZERO_THRESHOLD)
                && MathUtilities.SpeedUtilities.convertChassisSpeed(RobotContainer.swerveSubsystem
                        .getRobotVelocity()) < Units.feetToMeters(Constants.SwerveConstants.TRANSLATION_ZERO_THRESHOLD);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.swerveSubsystem.swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}
