package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.FileUtilities;
import frc.robot.utilities.JsonUtilities;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {

    public SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        makeYagslSwerveConfig();

        createSwerveObjects();
        
        System.out.println("Created SwerveSubsystem");
    }

    private void createSwerveObjects() {

        Pose2d startingPose = new Pose2d(new Translation2d(Units.feetToMeters(Constants.Starting.X), Units.feetToMeters(Constants.Starting.Y)), Rotation2d.fromDegrees(0)); // MAKE IT START ON OTHER SIDE WHEN RED OR WHAT EVER

        try {
            System.out.println("Initalizing swerveDrive");
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), Constants.SwerveConstants.SWERVECONFIGDIR)).createSwerveDrive(Units.feetToMeters(Constants.RobotKinematicConstants.MAX_SPEED), startingPose);

        } catch (Exception e)
        {
          throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, Constants.SwerveConstants.Imu.ANGULAR_VELOCITY_COEFF);
        swerveDrive.setGyroOffset(new Rotation3d(0,0, Units.degreesToRadians(Constants.SwerveConstants.Imu.OFFSET)));
    }

    private void makeYagslSwerveConfig() {
        System.out.println("Constructing YAGSL config objects");

        // Construct all the modules json based on the classes in Constants
        String backLeft = JsonUtilities.moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.BackLeft.class);
        String backRight = JsonUtilities.moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.BackRight.class);
        String frontLeft = JsonUtilities.moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.FrontLeft.class);
        String frontRight = JsonUtilities.moduleToJson(Constants.SwerveConstants.SwerveModuleConstants.FrontRight.class);

        String physicalProperties = JsonUtilities.physicalPropertiesToJson();
        String pidfProperties = JsonUtilities.pidfPropertiesToJson();
        String controllerProperties = JsonUtilities.controllerPropertiesToJson();
        String swerveDrive = JsonUtilities.swerveDriveToJson();

        System.out.println("Saving constructed YAGSL config objects");

        // Save the constructed objects
        FileUtilities.writeFile("swerve/modules/backleft.json", backLeft);
        FileUtilities.writeFile("swerve/modules/backright.json", backRight);
        FileUtilities.writeFile("swerve/modules/frontleft.json", frontLeft);
        FileUtilities.writeFile("swerve/modules/frontright.json", frontRight);

        FileUtilities.writeFile("swerve/modules/physicalproperties.json", physicalProperties);
        FileUtilities.writeFile("swerve/modules/pidfproperties.json", pidfProperties);
        FileUtilities.writeFile("swerve/controllerproperties.json", controllerProperties);
        FileUtilities.writeFile("swerve/swervedrive.json", swerveDrive);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOrientedSupplier(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
        swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
    
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public AHRS getGyro() {
        return (AHRS) swerveDrive.getGyro().getIMU();
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
        System.out.println("Zeroed gyro");
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        RobotContainer.visionSubsystem.updateVisionEstimates();
    }
}
