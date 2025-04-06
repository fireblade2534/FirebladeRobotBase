package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.MathUtilities;

public class SimulationSubsystem extends SubsystemBase {
    private double pickupAreaWidth = Units.feetToMeters(Constants.ArmConstants.Intake.Simulation.WIDTH);
    private double pickupAreaLength = Units.feetToMeters(Constants.ArmConstants.Intake.Simulation.LENGTH);
    private Pose3d endEffectorPose;
    private Pose3d swervePose;
    private boolean canPickup = false;

    public final IntakeIOSim intakeSim = new IntakeIOSim(pickupAreaWidth, pickupAreaLength);

    public SimulationSubsystem() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public boolean isPickupEnabled() {
        return canPickup;
    }

    @Override
    public void simulationPeriodic() {
        // Get the current position of the swerve acording to the simulation then calculate the end effector pose
        swervePose = new Pose3d(RobotContainer.swerveSubsystem.swerveDrive.getSimulationDriveTrainPose().get());
        endEffectorPose = calculateEndEffectorPose();

        simulatePickupZone();

        simulateEjectCoral();
    }

    private Pose3d calculateEndEffectorPose() {
        // Calculate where the end effector of the arm is
        Pose3d endEffectorPose = swervePose
                .plus(new Transform3d(Units.feetToMeters(Constants.ArmConstants.Shoulder.CENTER_OFFSET_FOWARD), 0,
                        RobotContainer.elevatorSubsystem.getCarpetElevatorHeight(),
                        new Rotation3d(0, -Units.degreesToRadians(RobotContainer.armSubsystem.getShoulderAngle()), 0)));

        // Add the length of the arm onto the end effector pose to get the positon at the end of the arm
        return endEffectorPose.plus(
                new Transform3d(Units.feetToMeters(Constants.ArmConstants.LENGTH), 0, 0, new Rotation3d(0, 0, 0)));
    }

    public Pose3d getEndCoralPose() {
        if (endEffectorPose != null) {
            Pose3d outputPose = new Pose3d(endEffectorPose.getTranslation(), new Rotation3d(0 ,0, Units.degreesToRadians(90) + endEffectorPose.getRotation().getZ()));

            outputPose = outputPose.plus(new Transform3d(0, 0, 0, new Rotation3d(endEffectorPose.getRotation().getY(), 0, 0)));

            outputPose = outputPose.plus(new Transform3d(0,0,0, new Rotation3d(0, -Units.degreesToRadians(RobotContainer.armSubsystem.getWristAngle()), 0 )));

            outputPose = outputPose.plus(new Transform3d(0, 0, -0.015, new Rotation3d()));

            return outputPose;
        }

        return new Pose3d();
    }

    private void simulateEjectCoral() {
        if (RobotContainer.intakeSubsystem.getIntakeSpeedPercent() > 0.2) {
            if (intakeSim.ejectCoral()) {
                System.out.println("Ejecting simulated piece");
                Pose3d coralPose = getEndCoralPose();
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(new CoralFlightSim(ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                        coralPose.getTranslation(), new Translation3d(0, 0, 0),
                        coralPose.getRotation()));

                /*
                AbstractDriveTrainSimulation driveSimulation = RobotContainer.swerveSubsystem.swerveDrive.getMapleSimDrive().get();
                SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                    // Obtain robot position from drive simulation
                    driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                    // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                    new Translation2d(0.6, 0),
                    // Obtain robot speed from drive simulation
                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    // Obtain robot facing from drive simulation
                    driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                    // The height at which the coral is ejected
                    coralPose.getTranslation().getMeasureZ(),
                    // The initial speed of the coral
                    MetersPerSecond.of(0),
                    // The coral is ejected vertically downwards
                    Degrees.of(0)));
                */
            }

        }
    }

    private void simulatePickupZone() {
        // Get the center of the pose in feild relative space
        Pose3d pickupAreaPose = swervePose.plus(new Transform3d(intakeSim.getShape().getCenter().x,
                intakeSim.getShape().getCenter().y, 0.05, new Rotation3d()));

        // Check if the end of the arm is close enough to pickup
        boolean inPickupZone = pickupAreaPose.getTranslation().getDistance(endEffectorPose.getTranslation()) < Math
                .max(pickupAreaWidth, pickupAreaLength);

        // Check if the intake is trying to intake coral and if the arm is in a position to pick up coral
        canPickup = inPickupZone && (RobotContainer.intakeSubsystem.getIntakeSpeedPercent() < -0.2);

        intakeSim.setRunning(canPickup);
    }
}
