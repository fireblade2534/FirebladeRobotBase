package frc.robot.utilities;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class MathUtilities {
    public static class ElevatorUtilities {

        /**
         * Convert rotations into distance
         * 
         * @param rotations rotations in degrees
         * @param drumRadius Drum radius in meters
         * @param gearRatio
         * @return distance
         */
        public static Distance convertRotationsToDistance(double rotations, double drumRadius, double gearRatio) {
            return Meters.of(rotations * ((drumRadius * 2 * Math.PI) / gearRatio));
        }

        /**
         * Convert distance into angles
         * 
         * @param distance Distance in meters
         * @param drumRadius Drum radius in meters
         * @param gearRatio
         * @return angle
         */
        public static Angle convertDistanceToRotations(double distance, double drumRadius, double gearRatio) {
            return Rotations.of((distance / (drumRadius * 2 * Math.PI)) * gearRatio);
        }
    }
}
