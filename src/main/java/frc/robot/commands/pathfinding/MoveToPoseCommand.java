package frc.robot.commands.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.MathUtilities;

public class MoveToPoseCommand extends Command {
    private final Pose2d goalPose;
    private final double translationErrorTolerance;
    private final double rotationErrorTolerance;

    private final ProfiledPIDController translationXController;
    private final ProfiledPIDController translationYController;
    private final PIDController headingController;

    /**
     * 
     * @param goalPose
     * @param translationErrorTolerance The tollerance in meters
     * @param rotationErrorTolerance The tollerance in degrees
     */
    public MoveToPoseCommand(Pose2d goalPose, double translationErrorTolerance, double rotationErrorTolerance) {
        this.goalPose = goalPose;
        this.translationErrorTolerance = translationErrorTolerance;
        this.rotationErrorTolerance = Units.degreesToRadians(rotationErrorTolerance);

        TrapezoidProfile.Constraints translatioConstraints = new TrapezoidProfile.Constraints(Units.feetToMeters(Constants.RobotKinematicConstants.MAX_SPEED), Units.feetToMeters(Constants.RobotKinematicConstants.MAX_ACCELERATION));

        this.translationXController = new ProfiledPIDController(Constants.SwerveConstants.TranslationPID.P, Constants.SwerveConstants.TranslationPID.I, Constants.SwerveConstants.TranslationPID.D, translatioConstraints);
        this.translationYController = new ProfiledPIDController(Constants.SwerveConstants.TranslationPID.P, Constants.SwerveConstants.TranslationPID.I, Constants.SwerveConstants.TranslationPID.D, translatioConstraints);
        this.headingController = new PIDController(Constants.SwerveConstants.HeadingPID.P, Constants.SwerveConstants.HeadingPID.I, Constants.SwerveConstants.HeadingPID.D);
        
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();

        this.translationXController.reset(robotPose.getX());
        this.translationYController.reset(robotPose.getY());
        this.headingController.reset();

        this.translationXController.setGoal(this.goalPose.getX());
        this.translationYController.setGoal(this.goalPose.getY());
        this.headingController.setSetpoint(this.goalPose.getRotation().getRadians());

        this.translationXController.setTolerance(this.translationErrorTolerance);
        this.translationYController.setTolerance(this.translationErrorTolerance);
        this.headingController.setTolerance(this.rotationErrorTolerance);

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        System.out.println("Moving to pose: " + this.goalPose.toString());
    }

    @Override
    public void execute() {
        Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();
        double translationX = this.translationXController.calculate(robotPose.getX());
        double translationY = this.translationYController.calculate(robotPose.getY());

        double heading = this.headingController.calculate(RobotContainer.swerveSubsystem.swerveDrive.getOdometryHeading().getRadians());

        RobotContainer.swerveSubsystem.swerveDrive.drive(new Translation2d(translationX, translationY), heading, true, false);
    }

    @Override
    public boolean isFinished() {
        return this.translationXController.atGoal() && this.translationYController.atGoal() && this.headingController.atSetpoint() && Math.abs(RobotContainer.swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond) < Units.degreesToRadians(Constants.SwerveConstants.ROTATION_ZERO_THRESHOLD) && MathUtilities.SpeedUtilities.convertChassisSpeed(RobotContainer.swerveSubsystem.getRobotVelocity()) < Units.feetToMeters(Constants.SwerveConstants.TRANSLATION_ZERO_THRESHOLD);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
