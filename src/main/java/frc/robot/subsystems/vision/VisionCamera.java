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

    // Simulation
    private PhotonCameraSim cameraSim;

    public VisionCamera(String cameraName, Transform3d cameraOffset, VisionSystemSim visionSim) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraOffset = cameraOffset;

        photonPoseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);

        if (Robot.isSimulation()) {
            SimCameraProperties cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(1280, 800, new Rotation2d(1.43117, 0.980875));
            cameraProperties.setCalibError(0, 0);
            cameraProperties.setFPS(50);
            cameraProperties.setAvgLatencyMs(50);
            cameraProperties.setLatencyStdDevMs(15);

            this.cameraSim = new PhotonCameraSim(this.camera, cameraProperties);
            this.cameraSim.enableDrawWireframe(true);
            visionSim.addCamera(this.cameraSim, this.cameraOffset);
        }

    }

    public Optional<EstimatedRobotPose> updateVision() {
        Optional<EstimatedRobotPose> robotEstimate = Optional.empty();
        // photonPoseEstimator.setReferencePose(RobotContainer.swerveSubsystem.swerveDrive.getPose());
        for (var result : camera.getAllUnreadResults()) {
            robotEstimate = photonPoseEstimator.update(result);
            updateEstimationStdDevs(robotEstimate, result.getTargets());
        }
        robotEstimate.ifPresent(est -> {
            Pose3d estPose = est.estimatedPose;
            SmartDashboard.putNumberArray("PhotonVision/" + cameraName + "/Robot Estimate", new double[] {
                    estPose.getX(),
                    estPose.getY(),
                    estPose.getZ(),
                    estPose.getRotation().getQuaternion().getW(),
                    estPose.getRotation().getQuaternion().getX(),
                    estPose.getRotation().getQuaternion().getY(),
                    estPose.getRotation().getQuaternion().getZ()
            });
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

        }

        if (Constants.DebugConstants.DEBUG_VISION == true) {
            SmartDashboard.putNumberArray("PhotonVision/" + cameraName + "/Current Std Deviations",
                    currentStdDevs.getData());
        }

    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }
}
