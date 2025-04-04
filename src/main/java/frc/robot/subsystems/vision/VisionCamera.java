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
import org.photonvision.targeting.PhotonPipelineResult;
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
    private Matrix<N3, N1> currentStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    // Camera properties
    private double effectiveRange;

    // Simulation
    private PhotonCameraSim cameraSim;

    // SmartDashboard
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

    public VisionCamera(String cameraName, Transform3d cameraOffset, int width, int height, int fps,
            double diagonal_fov, double average_pixel_error, double average_pixel_error_std_devs,
            double average_latency, double average_latency_std_devs, double effectiveRange, VisionSystemSim visionSim) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraOffset = cameraOffset;
        this.effectiveRange = effectiveRange;

        photonPoseEstimator = new PhotonPoseEstimator(Constants.APRIL_TAG_FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            SimCameraProperties cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(width, height, Rotation2d.fromDegrees(diagonal_fov));
            cameraProperties.setCalibError(average_pixel_error, average_pixel_error_std_devs);
            cameraProperties.setFPS(fps);
            cameraProperties.setAvgLatencyMs(average_latency);
            cameraProperties.setLatencyStdDevMs(average_latency_std_devs);

            this.cameraSim = new PhotonCameraSim(this.camera, cameraProperties);
            // this.cameraSim.enableDrawWireframe(true);
            visionSim.addCamera(this.cameraSim, this.cameraOffset);
        }

    }

    public void updateVision() {
        Optional<EstimatedRobotPose> robotEstimate = Optional.empty();

        photonPoseEstimator.setLastPose(RobotContainer.swerveSubsystem.getPose());

        List<PhotonPipelineResult> photonResults = camera.getAllUnreadResults();

        if (photonResults.isEmpty()) {
            return;
        }

        for (PhotonPipelineResult result : photonResults) {
            robotEstimate = photonPoseEstimator.update(result);
            updateEstimationStdDevs(robotEstimate, result.getTargets());

            robotEstimate.ifPresent(est -> {
                latestEstimatedPose = Optional.ofNullable(est);

                RobotContainer.swerveSubsystem.swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, getEstimationStdDevs());
            });
        }
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty() || targets.size() == 0) {
            currentStdDevs = Constants.VisionConstants.VISION_SINGLE_TAG_STD_DEVS;
        } else {
            Matrix<N3, N1> temporaryStdDevs = Constants.VisionConstants.VISION_SINGLE_TAG_STD_DEVS;
            int validTargets = 0;
            double averageDistance, averageAmbiguity;
            averageDistance = averageAmbiguity = 0;

            for (var target : targets) {
                Optional<Pose3d> targetPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (targetPose.isEmpty()) {
                     
                    continue;
                }

                validTargets++;
                averageAmbiguity += target.getPoseAmbiguity();
                averageDistance += targetPose.get().getTranslation().getDistance(new Translation3d(RobotContainer.swerveSubsystem.getPose().getTranslation()));
            }

            averageDistance /= validTargets;
            averageAmbiguity /= validTargets;

            if (validTargets > 1) {
                temporaryStdDevs = Constants.VisionConstants.VISION_MULTI_TAG_STD_DEVS;
            }

            if (validTargets == 1 && averageDistance > effectiveRange) {
                temporaryStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                double distanceMultiplier = Math.pow(averageDistance, 2)
                        / Constants.VisionConstants.TARGET_DISTANCE_STD_DEVS_DIVISOR;

                /*
                ChassisSpeeds robotVelocity = RobotContainer.swerveSubsystem.swerveDrive.getRobotVelocity();
                double translationVelocityMultiplier = Math.sqrt(
                        Math.pow(robotVelocity.vxMetersPerSecond, 2) +
                                Math.pow(robotVelocity.vyMetersPerSecond, 2))
                        / Constants.VisionConstants.TARGET_TRANSLATION_SPEED_STD_DEVS_DIVISOR;
                double rotationalVelocityMultiplier = Math.abs(Units.radiansToDegrees(robotVelocity.omegaRadiansPerSecond))
                        / Constants.VisionConstants.TARGET_ROTATIONAL_SPEED_STD_DEVS_DIVISOR;

                double targetCountMultiplier = Math.max(
                        ((1 / validTargets) / Constants.VisionConstants.TARGET_COUNT_STD_DEVS_DIVISOR - 0), 0);

                double targetAmbiguity = averageAmbiguity / Constants.VisionConstants.TARGET_AMBIGUITY_STD_DEVS_DIVISOR;

                temporaryStdDevs = temporaryStdDevs.times(1 + ((distanceMultiplier + translationVelocityMultiplier
                        + rotationalVelocityMultiplier + targetCountMultiplier + targetAmbiguity) * Constants.VisionConstants.OVERALL_MULTIPLIER));
                
                */

                temporaryStdDevs.times(1 + distanceMultiplier);
                if (temporaryStdDevs.get(0, 0) > Constants.VisionConstants.CUTOFF) {
                    temporaryStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                }
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

    public Optional<EstimatedRobotPose> getLatestEstimatedPose() {
        return latestEstimatedPose;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }
}
