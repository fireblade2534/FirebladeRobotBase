package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionCamera;

public class SmartDashboardSubsystem extends SubsystemBase {
    public SmartDashboardSubsystem() {

    }

    @Override
    public void periodic() {
        if (Constants.DebugConstants.DEBUG_VISION == true) {

            for (VisionCamera camera : RobotContainer.visionSubsystem.limelightCameras) {
                SmartDashboard.putNumberArray("PhotonVision/" + camera.getCameraName() + "/Current Std Deviations",
                        camera.getEstimationStdDevs().getData());
                        
                Pose3d estimatedPose = camera.getLatestEstimatedPose().estimatedPose;
                SmartDashboard.putNumberArray("PhotonVision/" + camera.getCameraName() + "/Robot Estimated Pose",
                        new double[] {
                                estimatedPose.getX(),
                                estimatedPose.getY(),
                                estimatedPose.getZ(),
                                estimatedPose.getRotation().getQuaternion().getW(),
                                estimatedPose.getRotation().getQuaternion().getX(),
                                estimatedPose.getRotation().getQuaternion().getY(),
                                estimatedPose.getRotation().getQuaternion().getZ()
                        });
            }
        }
    }
}
