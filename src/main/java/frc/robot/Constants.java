// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class AdvantageKitConstants {
    public static final RobotMode SIMMODE = RobotMode.SIM;
    public static final RobotMode CURRENTMODE = RobotBase.isReal() ? RobotMode.REAL : SIMMODE;

    public static enum RobotMode {
      REAL, SIM, REPLAY
    }
  }

  public static class SwerveConstants {

    public static class SwerveModuleConstants {
      public static class BackLeft {
        public static class Location {
          public static final double FRONT = -14.75; // Inches
          public static final double LEFT = 9.875; // Inches
        }

        public static final double ABSOLUTE_ENCODER_OFFSET = 308.8381620; // Degrees

        public static class Drive {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 23;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }

        public static class Angle {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 33;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = true;
        }

        public static class Encoder {
          public static final String TYPE = "thrifty";
          public static final int ID = 3;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }
      }

      public static class BackRight {
        public static class Location {
          public static final double FRONT = -14.75; // Inches
          public static final double LEFT = -9.875; // Inches
        }

        public static final double ABSOLUTE_ENCODER_OFFSET = 150.9974208105; // Degrees

        public static class Drive {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 22;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }

        public static class Angle {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 32;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = true;
        }

        public static class Encoder {
          public static final String TYPE = "thrifty";
          public static final int ID = 2;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }
      }

      public static class FrontLeft {
        public static class Location {
          public static final double FRONT = 14.75; // Inches
          public static final double LEFT = 9.875; // Inches
        }

        public static final double ABSOLUTE_ENCODER_OFFSET = 19.47493551; // Degrees

        public static class Drive {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 20;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }

        public static class Angle {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 30;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = true;
        }

        public static class Encoder {
          public static final String TYPE = "thrifty";
          public static final int ID = 0;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }
      }

      public static class FrontRight {
        public static class Location {
          public static final double FRONT = 14.75; // Inches
          public static final double LEFT = -9.875; // Inches
        }

        public static final double ABSOLUTE_ENCODER_OFFSET = 112.15935540; // Degrees

        public static class Drive {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 21;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }

        public static class Angle {
          public static final String TYPE = "sparkmax_neo";
          public static final int ID = 31;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = true;
        }

        public static class Encoder {
          public static final String TYPE = "thrifty";
          public static final int ID = 1;
          public static final Object CANBUS = null;
          public static final boolean INVERTED = false;
        }
      }
    }

    public static class Imu {
      public static final String TYPE = "navx";
      public static final int ID = 0;
      public static final Object CANBUS = null;
      public static final boolean INVERTED = false;
    }

    public static class MotorConstants {
      public static class Angle {
        public static final double RAMP_RATE = 0.25;
        public static final double GEAR_RATIO = 21.4285714286;
        public static final double FACTOR = 0;
        public static final int CURRENT_LIMIT = 20; // Amps

        public static class Pidf {
          public static final double P = 0.01;
          public static final double I = 0;
          public static final double D = 0.1;
          public static final double F = 0;
          public static final double IZ = 0;
        }
      }

      public static class Drive {
        public static final double RAMP_RATE = 0.25;
        public static final double GEAR_RATIO = 6.75;
        public static final double FACTOR = 0;
        public static final int CURRENT_LIMIT = 40; // Amps

        public static class Pidf {
          public static final double P = 0.0020645;
          public static final double I = 0;
          public static final double D = 0;
          public static final double F = 0;
          public static final double IZ = 0;
        }
      }

      public static final int OPTIMAL_VOLTAGE = 12; // Volts
    }

    public static class HeadingPid {
      public static final double P = 0.4;
      public static final double I = 0;
      public static final double D = 0.01;
    }

    public static class WheelConstants {
      public static final double WHEEL_GRIP_COF = 1.19;
      public static final double DIAMETER = 4; // Inches
    }

    public static final boolean ENABLE_FEED_FOWARD = true; // Controls if feed foward should be enabled in the audo
                                                           // builder
    public static final double ANGLE_JOYSTICK_RADIUS_DEADBAND = 0.5;
    public static final List<String> MODULE_FILES = List.of("frontleft.json", "frontright.json", "backleft.json",
        "backright.json");
    public static final String SWERVECONFIGDIR = "swerve";
  }

  public static class RobotKinematicConstants {
    public static final double MASS = 115; // Pounds
    public static final double WIDTH = 2.58333333; // Feet
    public static final double LENGTH = 3.41666667; // Feet
    public static final double MAX_SPEED = 24; // Feet/Seconds
    public static final double MAX_ACHIEVABLE_SPEED = 24; // Feet/Seconds
    public static final double MOI = 6.883;
    public static final String DRIVE_MOTOR_TYPE = "NEO";

    public static class BumperOffset {
      public static final double X = 0; // Feet
      public static final double Y = 0; // Feet
    }
  }

  public static class DriverConstants {
    public static final int PORT = 0;
    public static final double DEADBAND = 0.1;
    public static final double TRANSLATION_SCALE = 1;
    public static final double ROTATION_SCALE = 1;
    public static final int CONTROL_EXPONENT = 2;
  }

  public static class VisionConstants {
    public static final boolean USE_WELDED_FIELD = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(USE_WELDED_FIELD ? AprilTagFields.k2025ReefscapeWelded : AprilTagFields.k2025ReefscapeAndyMark);
    public static final Matrix<N3, N1> VISION_SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.45); // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> VISION_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 0.2); // The standard deviations of our vision estimated poses, which affect correction rate
    public static final double TARGET_DISTANCE_STD_DEVS_DIVISOR = 100; // The higher this is the less that far targets increase the std devs
    public static final double TARGET_TRANSLATION_SPEED_STD_DEVS_DIVISOR = 150; // The higher this is the less that the robot moving increases the std devs
    public static final double TARGET_ROTATIONAL_SPEED_STD_DEVS_DIVISOR = 150; // The higher this is the less that the robot turning increases the std devs
    public static final double TARGET_COUNT_STD_DEVS_DIVISOR = 4; // The higher this is the less that a low number of targets increases the std devs

    public static class Limelight_4 {
      public static final String NAME = "Limelight 4";
      public static final boolean ENABLED = true;
      public static final double FRONT_OFFSET = 0; // Feet
      public static final double LEFT_OFFSET = -1.31234; // Feet
      public static final double HEIGHT_OFFSET = 2.95276; // Feet
      public static final double ROLL = 0; // Degrees
      public static final double PITCH = 0; // Degrees
      public static final double YAW = 20; // Degrees
      public static final double EFFECTIVE_RANGE = 5; // Meters

      public static class CameraProperties {
        public static final int WIDTH = 1280; // Pixels
        public static final int HEIGHT = 800; // Pixels
        public static final int FPS = 120;
        public static final double DIAGONAL_FOV = 91.12; // Degrees
        public static final double AVERGAGE_PIXEL_ERROR = 0.25;
        public static final double AVERGAGE_PIXEL_ERROR_STD_DEVS = 0.05;
        public static final double AVERAGE_LATENCY = 15; // Miliseconds
        public static final double AVERAGE_LATENCY_STD_DEVS = 0.1;
      }
    }
  }

  public static class DebugConstants {
    public static boolean DEBUG_VISION = true;
  }
}
