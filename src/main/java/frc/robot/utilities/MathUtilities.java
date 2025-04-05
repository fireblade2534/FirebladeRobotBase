package frc.robot.utilities;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class MathUtilities {
    
    public static class ArmUtilities {

        /**
         * Convert the actual angle of the arm to the motor encoder
         * 
         * @param rotations rotations
         * @param gearRatio
         * @return
         */
        public static Angle convertArmAngleToMotorAngle(double rotations, double gearRatio) {
            return Rotations.of(rotations * gearRatio);
        }

        /**
         * Convert the encoder output to the actual angle of the arm
         * 
         * @param rotations rotations
         * @param gearRatio
         * @return
         */
        public static Angle convertMotorAngleToArmAngle(double rotations, double gearRatio) {
            return Rotations.of(rotations / gearRatio);
        }
    }

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

    public static class PoseUtilities {
        public static Rotation2d rotationToTarget(Translation2d source, Translation2d target) {
            double targetRadians = Math.atan2(target.getY() - source.getY(), target.getX() - source.getX());
            return new Rotation2d(targetRadians);
        }
    }

    public static class SpeedUtilities {
        public static double convertChassisSpeed(ChassisSpeeds chassisSpeeds) {
            return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        }
    }

    public static class AngleUtilities {
        public static double normalizeAngle(double degrees) {
            degrees = degrees % 360;

            if (degrees > 180) {
                degrees -= 360;
            } else if (degrees < -180) {
                degrees += 360;
            }

            return degrees;
        }

        public static Angle getPerpendicularAngle(double degrees) {
            return Degrees.of(normalizeAngle(degrees - 90));
        }
    }
}
