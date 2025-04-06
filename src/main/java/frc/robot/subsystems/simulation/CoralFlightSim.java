package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * 
 * CoralFlightSim is a modified version of {@link ReefscapeCoralOnFly}
 */
public class CoralFlightSim extends GamePieceProjectile {
    
    public CoralFlightSim(GamePieceOnFieldSimulation.GamePieceInfo info,
            Translation3d initialPosition,
            Translation3d initialLaunchingVelocityMPS,
            Rotation3d gamePieceRotation) {
        super(info, initialPosition.toTranslation2d(), initialLaunchingVelocityMPS.toTranslation2d(), initialPosition.getZ(), initialLaunchingVelocityMPS.getZ(), gamePieceRotation);

        super.enableBecomesGamePieceOnFieldAfterTouchGround();
        super.withTouchGroundHeight(0.2);
    }

    @Override
    public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
        if (!super.becomesGamePieceOnGroundAfterTouchGround) return;
        System.out.println(super.launchedTimer.get());
        System.out.println(getPositionAtTime(super.launchedTimer.get()).getZ());
        simulatedArena.addGamePiece(new GamePieceOnFieldSimulation(
                ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                () -> Math.max(
                        ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO
                                        .gamePieceHeight()
                                        .in(Meters)
                                ,
                        getPositionAtTime(super.launchedTimer.get()).getZ()),
                new Pose2d(
                        getPositionAtTime(launchedTimer.get()).toTranslation2d(),
                        super.gamePieceRotation.toRotation2d()),
                super.initialLaunchingVelocityMPS));
    }
}
