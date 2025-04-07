package frc.robot.subsystems.simulation;

import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Shape;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeIOSim extends IntakeSimulation {

    public IntakeIOSim(double width, double height) {
        super("Coral", RobotContainer.swerveSubsystem.swerveDrive.getMapleSimDrive().get(),
            constructIntakeRectangle(RobotContainer.swerveSubsystem.swerveDrive.getMapleSimDrive().get(), width, height),
                1);
        
        if (Constants.ArmConstants.Intake.Simulation.ASSUME_START_WITH) {
            super.addGamePieceToIntake();
        }
    }

    /**
     * This is a simpler version of {@link IntakeSimulation}'s getIntakeRectangle
     */
    private static Rectangle constructIntakeRectangle(AbstractDriveTrainSimulation driveTrainSimulation, double width, double length) {
        Rectangle intakeRectangle = new Rectangle(width, length);

        intakeRectangle.rotate(Math.toRadians(90));

        double distanceTransformed = length / 2 - 0.01;

        intakeRectangle.translate(
            driveTrainSimulation.config.bumperLengthX.in(Meters) / 2 + distanceTransformed, 0);

        return intakeRectangle;
    }

    public void setRunning(boolean running) {
        if (running) {
            super.startIntake();
        } else {
            super.stopIntake();
        }
    }

    public boolean containsCoral() {
        return super.getGamePiecesAmount() != 0;
    }

    public boolean ejectCoral() {
        return super.obtainGamePieceFromIntake();
    }

    public Convex getShape() {
        return super.shape;
    }
}
