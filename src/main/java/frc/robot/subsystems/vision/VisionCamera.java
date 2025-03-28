package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class VisionCamera {
    private PhotonCamera camera;
    private String cameraName;
    private Transform3d cameraOffset;
    private PhotonPoseEstimator photonPoseEstimator;
    private Matrix<N3, N1> currentStdDevs;
    // Camera properties
    private double effectiveRange;

    // Simulation
    private PhotonCameraSim cameraSim;

    // SmartDashboard
    private EstimatedRobotPose latestEstimatedPose;

    public VisionCamera(String cameraName, Transform3d cameraOffset, int width, int height, int fps,
            double diagonal_fov, double average_pixel_error, double average_pixel_error_std_devs,
            double average_latency, double average_latency_std_devs, double effectiveRange, VisionSystemSim visionSim) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraOffset = cameraOffset;
        this.effectiveRange = effectiveRange;

        photonPoseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);

        if (Robot.isSimulation()) {
            SimCameraProperties cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(width, height, Rotation2d.fromDegrees(diagonal_fov));
            cameraProperties.setCalibError(average_pixel_error, average_latency_std_devs);
            cameraProperties.setFPS(fps);
            cameraProperties.setAvgLatencyMs(average_latency);
            cameraProperties.setLatencyStdDevMs(average_latency_std_devs);

            this.cameraSim = new PhotonCameraSim(this.camera, cameraProperties);
            this.cameraSim.enableDrawWireframe(true);
            visionSim.addCamera(this.cameraSim, this.cameraOffset);
        }

    }

    public Optional<EstimatedRobotPose> updateVision() {
        Optional<EstimatedRobotPose> robotEstimate = Optional.empty();

        for (var result : camera.getAllUnreadResults()) {
            robotEstimate = photonPoseEstimator.update(result);
            updateEstimationStdDevs(robotEstimate, result.getTargets());
        }
        robotEstimate.ifPresent(est -> {
            latestEstimatedPose = est;
        });
        return robotEstimate;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty() || targets.size() == 0) {
            currentStdDevs = Constants.VisionConstants.VISION_SINGLE_TAG_STD_DEVS;
        } else {
            Translation3d estimatedTranslation = estimatedPose.get().estimatedPose.getTranslation();
            Matrix<N3, N1> temporaryStdDevs = Constants.VisionConstants.VISION_SINGLE_TAG_STD_DEVS;
            int validTargets = 0;
            double averageDistance = 0;

            for (var target : targets) {
                Optional<Pose3d> targetPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (targetPose.isEmpty()) {
                    continue;
                }

                validTargets++;
                averageDistance += targetPose.get().getTranslation().getDistance(estimatedTranslation);
            }

            averageDistance /= validTargets;

            if (validTargets > 1) {
                temporaryStdDevs = Constants.VisionConstants.VISION_MULTI_TAG_STD_DEVS;
            }

            if (validTargets == 1 && averageDistance > effectiveRange) {
                temporaryStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                double distanceMultiplier = Math.pow(averageDistance, 2)
                        / Constants.VisionConstants.TARGET_DISTANCE_STD_DEVS_DIVISOR;

                ChassisSpeeds robotVelocity = RobotContainer.swerveSubsystem.swerveDrive.getRobotVelocity();
                double translationVelocityMultiplier = Math.sqrt(
                        Math.pow(robotVelocity.vxMetersPerSecond, 2) +
                                Math.pow(robotVelocity.vyMetersPerSecond, 2))
                        / Constants.VisionConstants.TARGET_TRANSLATION_SPEED_STD_DEVS_DIVISOR;
                double rotationalVelocityMultiplier = Units.radiansToDegrees(robotVelocity.omegaRadiansPerSecond)
                        / Constants.VisionConstants.TARGET_ROTATIONAL_SPEED_STD_DEVS_DIVISOR;

                double targetCountMultiplier = Math.max(
                        ((2 / validTargets) / Constants.VisionConstants.TARGET_COUNT_STD_DEVS_DIVISOR - 0.2), -0.2);

                temporaryStdDevs = temporaryStdDevs.times(1 + distanceMultiplier + translationVelocityMultiplier
                        + rotationalVelocityMultiplier + targetCountMultiplier);
            }

            currentStdDevs = temporaryStdDevs;
        }

    }

    public String getCameraName() {
        return cameraName;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public EstimatedRobotPose getLatestEstimatedPose() {
        return latestEstimatedPose;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }
}
