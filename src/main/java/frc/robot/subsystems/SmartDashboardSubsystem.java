package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.utilities.MathUtilities;

public class SmartDashboardSubsystem extends SubsystemBase {
    public SmartDashboardSubsystem() {
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput("FieldSimulation/Algae", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

        if (Constants.DebugConstants.DEBUG_SIMULATION) {
            SmartDashboard.putBoolean("Simulation/Intake/Enabled", RobotContainer.simulationSubsystem.isPickupEnabled()); 
        }

        if (Constants.DebugConstants.ANIMATE_ROBOT) {
            if (RobotContainer.simulationSubsystem.intakeSim.containsCoral()) {
                SmartDashboard.putNumberArray("Simulation/Animate/Coral", MathUtilities.PoseUtilities.convertPose3dToNumbers(RobotContainer.simulationSubsystem.getEndCoralPose()));
            } else {
                SmartDashboard.putNumberArray("Simulation/Animate/Coral", MathUtilities.PoseUtilities.convertPose3dToNumbers(new Pose3d()));
            }
            
        }
    }

    @Override
    public void periodic() {
        // Stuff here should always be sent to maintain basic functionality
        SmartDashboard.putBoolean("Arm/Intake/HasCoral", RobotContainer.intakeSubsystem.hasCoral());

        if (Constants.DebugConstants.DEBUG_VISION == true) {

            for (VisionCamera camera : RobotContainer.visionSubsystem.limelightCameras) {
                SmartDashboard.putNumberArray("PhotonVision/" + camera.getCameraName() + "/Current Std Deviations",
                        camera.getEstimationStdDevs().getData());

                EstimatedRobotPose estimatedRobotPose = camera.getLatestEstimatedPose();
                if (estimatedRobotPose != null) {
                    SmartDashboard.putNumberArray("PhotonVision/" + camera.getCameraName() + "/Robot Estimated Pose", MathUtilities.PoseUtilities.convertPose3dToNumbers(estimatedRobotPose.estimatedPose));
                }
            }
        }
        
        if (Constants.DebugConstants.DEBUG_INTAKE == true) {
            SmartDashboard.putNumber("Arm/Intake/IntakeMotorPercent", RobotContainer.intakeSubsystem.getIntakeSpeedPercent());
        }

        if (Constants.DebugConstants.DEBUG_ELEVATOR || Constants.DebugConstants.DEBUG_ARM || Constants.DebugConstants.DEBUG_WRIST || Constants.DebugConstants.ANIMATE_ROBOT) {

            // Get a bunch of values that can be used multiple times
            double stage1Height = RobotContainer.elevatorSubsystem.getStage1Height();
            double stage2Height = RobotContainer.elevatorSubsystem.getStage2Height();

            double shoulderAngle = RobotContainer.armSubsystem.getShoulderAngle();

            double wristAngle = RobotContainer.armSubsystem.getWristAngle();

            if (Constants.DebugConstants.DEBUG_ELEVATOR) {
                SmartDashboard.putNumber("Elevator/Stage1/Encoder", RobotContainer.elevatorSubsystem.getStage1MotorEncoder());
                SmartDashboard.putNumber("Elevator/Stage2/Encoder", RobotContainer.elevatorSubsystem.getStage2MotorEncoder());

                SmartDashboard.putNumber("Elevator/Stage1/Setpoint", RobotContainer.elevatorSubsystem.getStage1Setpoint());
                SmartDashboard.putNumber("Elevator/Stage2/Setpoint", RobotContainer.elevatorSubsystem.getStage2Setpoint());
                
                SmartDashboard.putNumber("Elevator/Stage1/Height", stage1Height);
                SmartDashboard.putNumber("Elevator/Stage2/Height", stage2Height);
            }

            if (Constants.DebugConstants.DEBUG_ARM) {
                SmartDashboard.putNumber("Arm/Shoulder/Setpoint", RobotContainer.armSubsystem.getShoulderSetpoint());
                SmartDashboard.putNumber("Arm/Shoulder/Angle", shoulderAngle);
            }

            if (Constants.DebugConstants.DEBUG_WRIST) {
                SmartDashboard.putNumber("Arm/Wrist/Setpoint", RobotContainer.armSubsystem.getWristSetpoint());
                SmartDashboard.putNumber("Arm/Wrist/Angle", wristAngle);
            }

            if (Constants.DebugConstants.ANIMATE_ROBOT) {
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

                Pose3d shoulderPose = new Pose3d(Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD), 0, stage1Height + stage2Height + RobotContainer.elevatorSubsystem.getPivotPointOffset(false), new Rotation3d(0, -Units.degreesToRadians(shoulderAngle),0));
                SmartDashboard.putNumberArray("Arm/Shoulder/Position", MathUtilities.PoseUtilities.convertPose3dToNumbers(shoulderPose));

                Pose3d wristPose = new Pose3d(Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD), 0, stage1Height + stage2Height + RobotContainer.elevatorSubsystem.getPivotPointOffset(false), new Rotation3d(Units.degreesToRadians(wristAngle), -Units.degreesToRadians(shoulderAngle),0));
                SmartDashboard.putNumberArray("Arm/Wrist/Position", MathUtilities.PoseUtilities.convertPose3dToNumbers(wristPose));
                
            }
        }

    }
}
