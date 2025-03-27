package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.VisionSystemSim;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    private AHRS navx;

    private List<VisionCamera> limelightCameras = new ArrayList<>();

    public VisionSystemSim visionSim;

    public VisionSubsystem() {
        navx = (AHRS) RobotContainer.swerveSubsystem.swerveDrive.getGyro().getIMU();

        if (Robot.isSimulation()) {

            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT);
        }

        for (Class<?> limelightConfig : Constants.VisionConstants.class.getDeclaredClasses()) {
            String limelightName;
            try {
                limelightName = (String) limelightConfig.getDeclaredField("NAME").get(null);
                if ((boolean) limelightConfig.getDeclaredField("ENABLED").get(null) == true) {

                    Translation3d limelightRelativePosition = new Translation3d(
                            Units.feetToMeters((double) limelightConfig.getDeclaredField("FRONT_OFFSET").get(null)),
                            Units.feetToMeters((double) limelightConfig.getDeclaredField("LEFT_OFFSET").get(null)),
                            Units.feetToMeters((double) limelightConfig.getDeclaredField("HEIGHT_OFFSET").get(null)));

                    Rotation3d limelightRelativeRotation = new Rotation3d(
                            Units.degreesToRadians(
                                    (double) limelightConfig.getDeclaredField("ROLL").get(null)),
                            Units.degreesToRadians(
                                    (double) limelightConfig.getDeclaredField("PITCH").get(null)),
                            Units.degreesToRadians(
                                    (double) limelightConfig.getDeclaredField("YAW").get(null)));

                    System.out.println(limelightRelativeRotation.toString());
                    limelightCameras.add(new VisionCamera(limelightName,
                            new Transform3d(limelightRelativePosition, limelightRelativeRotation),
                            visionSim));

                    System.out.println("Limelight with name " + limelightName + " is enabled");
                } else {
                    System.out.println("Limelight with name " + limelightName + " is not enabled");
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().ifPresent(pose -> {
                visionSim.update(pose);
            });
            
        }

        for (var camera : limelightCameras) {
            var visionEstimate = camera.updateVision();

            visionEstimate.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = camera.getEstimationStdDevs();

                    RobotContainer.swerveSubsystem.swerveDrive.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
        }
    }
}
