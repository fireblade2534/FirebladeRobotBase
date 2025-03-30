package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionCamera;

public class SmartDashboardSubsystem extends SubsystemBase {
    public SmartDashboardSubsystem() {
        SmartDashboard.putData(CommandScheduler.getInstance());

        
    }

    @Override
    public void periodic() {
        if (Constants.DebugConstants.DEBUG_VISION == true) {

            for (VisionCamera camera : RobotContainer.visionSubsystem.limelightCameras) {
                SmartDashboard.putNumberArray("PhotonVision/" + camera.getCameraName() + "/Current Std Deviations",
                        camera.getEstimationStdDevs().getData());

                camera.getLatestEstimatedPose().ifPresent(latestPose -> {
                    Pose3d estimatedPose = latestPose.estimatedPose;
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
                });
            }
        }


        if (Constants.DebugConstants.DEBUG_ELEVATOR) {
            SmartDashboard.putNumber("Elevator/Stage1/Encoder", RobotContainer.elevatorSubsystem.getStage1MotorEncoder());
            SmartDashboard.putNumber("Elevator/Stage2/Encoder", RobotContainer.elevatorSubsystem.getStage2MotorEncoder());

            SmartDashboard.putNumber("Elevator/Stage1/Setpoint", RobotContainer.elevatorSubsystem.getStage1EncoderSetpoint());
            SmartDashboard.putNumber("Elevator/Stage2/Setpoint", RobotContainer.elevatorSubsystem.getStage2EncoderSetpoint());
        }

        if (Constants.DebugConstants.ANIMATE_ROBOT) {
            
            double stage1Height = RobotContainer.elevatorSubsystem.getStage1Height();
            double stage2Height = RobotContainer.elevatorSubsystem.getStage2Height();
            SmartDashboard.putNumberArray("Elevator/Stage1/Position", new double[] {
                0,
                0,
                stage1Height,
                0,
                0,
                0,
                0
            });

            SmartDashboard.putNumberArray("Elevator/Stage2/Position", new double[] {
                0,
                0,
                stage1Height + stage2Height,
                0,
                0,
                0,
                0
            });

            SmartDashboard.putNumberArray("Elevator/Shoulder/Position", new double[] {
                0,
                0,
                stage1Height + stage2Height,
                0,
                0,
                0,
                0
            });
            
        }

    }
}
