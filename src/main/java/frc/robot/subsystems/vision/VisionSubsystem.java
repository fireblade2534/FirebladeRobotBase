package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.VisionSystemSim;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.JsonUtilities;

public class VisionSubsystem extends SubsystemBase {

    public AHRS navx;

    public List<VisionCamera> limelightCameras = new ArrayList<>();

    public VisionSystemSim visionSim;

    public VisionSubsystem() {
        navx = (AHRS) RobotContainer.swerveSubsystem.swerveDrive.getGyro().getIMU();

        if (Robot.isSimulation()) {

            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Constants.APRIL_TAG_FIELD_LAYOUT);
        }

        boolean anyCamera = setupCameras();
        if (anyCamera) {
            RobotContainer.swerveSubsystem.swerveDrive.stopOdometryThread();
        }
    }

    private boolean setupCameras() {
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

                    double effectiveRange = Units
                            .feetToMeters((double) limelightConfig.getDeclaredField("EFFECTIVE_RANGE").get(null));

                    Class<?> cameraProperties = JsonUtilities.getInnerClass(limelightConfig, "CameraProperties");

                    int width = (int) cameraProperties.getDeclaredField("WIDTH").get(null);
                    int height = (int) cameraProperties.getDeclaredField("HEIGHT").get(null);
                    int fps = (int) cameraProperties.getDeclaredField("FPS").get(null);
                    double diagonal_fov = (double) cameraProperties.getDeclaredField("DIAGONAL_FOV").get(null);

                    double average_pixel_error = (double) cameraProperties.getDeclaredField("AVERGAGE_PIXEL_ERROR")
                            .get(null);
                    double average_pixel_error_std_devs = (double) cameraProperties
                            .getDeclaredField("AVERGAGE_PIXEL_ERROR_STD_DEVS").get(null);

                    double average_latency = (double) cameraProperties.getDeclaredField("AVERAGE_LATENCY").get(null);
                    double average_latency_std_devs = (double) cameraProperties
                            .getDeclaredField("AVERAGE_LATENCY_STD_DEVS").get(null);

                    limelightCameras.add(new VisionCamera(limelightName,
                            new Transform3d(limelightRelativePosition, limelightRelativeRotation), width, height, fps,
                            diagonal_fov, average_pixel_error, average_pixel_error_std_devs, average_latency,
                            average_latency_std_devs, effectiveRange,
                            visionSim));

                    System.out.println("Camera with name " + limelightName + " is enabled");
                } else {
                    System.out.println("Camera with name " + limelightName + " is not enabled");
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return limelightCameras.size() > 0;
    }

    @Override
    public void simulationPeriodic() {
        // Update the vision simulation with the exact robot pose from the drive train
        // simulation
        RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().ifPresent(pose -> {
            visionSim.update(pose);
        });
    }


    public void updateVisionEstimates() {
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
