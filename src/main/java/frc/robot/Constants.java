// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
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

  public static final boolean USE_WELDED_FIELD = false;
  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(USE_WELDED_FIELD ? AprilTagFields.k2025ReefscapeWelded : AprilTagFields.k2025ReefscapeAndyMark);

  public static class Starting {
    public static final double X = 24.95316667; // Feet
    public static final double Y = Units.metersToFeet(APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2); // Feet
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
          public static final double P = 0.015;
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
          public static final double P = 0.0021645;
          public static final double I = 0;
          public static final double D = 0;
          public static final double F = 0;
          public static final double IZ = 0;
        }
      }

      public static final int OPTIMAL_VOLTAGE = 12; // Volts
    }

    public static class TranslationPID {
      public static final double P = 8;
      public static final double I = 0;
      public static final double D = 0.05;
    }

    public static class HeadingPID {
      public static final double P = 7;
      public static final double I = 0;
      public static final double D = 0.05;
    }

    public static class WheelConstants {
      public static final double WHEEL_GRIP_COF = 1.19;
      public static final double DIAMETER = 4; // Inches
    }

    public static final double TRANSLATION_ZERO_THRESHOLD = 0.05; // Feet/Second
    public static final double ROTATION_ZERO_THRESHOLD = 0.05; // Degrees/Second


    public static final boolean ENABLE_FEED_FOWARD = true; // Controls if feed foward should be enabled in the auto
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
    public static final double HEIGHT_OFF_GROUND = 0.033; // Feet
    public static final double MAX_SPEED = 24; // Feet/Second
    public static final double MAX_ACHIEVABLE_SPEED = 24; // Feet/Second
    public static final double MAX_ACCELERATION = 7.5; // Feet/Second
    public static final double MAX_ANGULAR_VELOCITY = 540; // Degrees/Second
    public static final double MAX_ANGULAR_ACCELERATION = 720; // Degrees/Second
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
    public static final double ROTATION_SCALE = 0.7;
    public static final int CONTROL_EXPONENT = 2;
    public static final double CONTROL_ELEVATOR_SPEED = 1; // Feet/Second
    public static final double CONTROL_SHOULDER_SPEED = 50; // Degrees/Second
    public static final double CONTROL_WRIST_SPEED = 60; // Degrees/Second
    public static final double INTAKE_SPEED = -0.9; // Percent
    public static final double OUTTAKE_SPEED = 0.9; // Percent

    public static class Alerts {
      public static final double LOW_BATTERY_VOLTAGE = 11.5; // Volts
    }
  }

  public static class VisionConstants {
    public static final Matrix<N3, N1> VISION_SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.6); // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> VISION_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.3); // The standard deviations of our vision estimated poses, which affect correction rate
    public static final double TARGET_DISTANCE_STD_DEVS_DIVISOR = 60; // The higher this is the less that far targets increase the std devs
    public static final double TARGET_TRANSLATION_SPEED_STD_DEVS_DIVISOR = 70; // The higher this is the less that the robot moving increases the std devs
    public static final double TARGET_ROTATIONAL_SPEED_STD_DEVS_DIVISOR = 70; // The higher this is the less that the robot turning increases the std devs
    public static final double TARGET_COUNT_STD_DEVS_DIVISOR = 0.2; // The higher this is the less that a low number of targets increases the std devs
    public static final double TARGET_AMBIGUITY_STD_DEVS_DIVISOR = 0.1; // The higher this is the less that a high average ambiguty increases the std devs
    public static final double OVERALL_MULTIPLIER = 1;
    public static final double CUTOFF = 2.5;

    public static class Limelight_4 {
      public static final String NAME = "Limelight 4";
      public static final boolean ENABLED = true;
      public static final double FRONT_OFFSET = 0; // Feet
      public static final double LEFT_OFFSET = -1.31234; // Feet
      public static final double HEIGHT_OFFSET = 2.95276; // Feet
      public static final double ROLL = 0; // Degrees
      public static final double PITCH = 0; // Degrees
      public static final double YAW = 28; // Degrees
      public static final double EFFECTIVE_RANGE = 6; // Meters

      public static class CameraProperties {
        public static final int WIDTH = 1280; // Pixels
        public static final int HEIGHT = 800; // Pixels
        public static final int FPS = 120;
        public static final double DIAGONAL_FOV = 91.12; // Degrees
        public static final double AVERGAGE_PIXEL_ERROR = 0.25;
        public static final double AVERGAGE_PIXEL_ERROR_STD_DEVS = 0.06;
        public static final double AVERAGE_LATENCY = 15; // Miliseconds
        public static final double AVERAGE_LATENCY_STD_DEVS = 0.1;
      }
    }
  }

  public static class ReefConstants {
      public static class FieldConstants {
          public static final int[] BLUE_ALLIANCE_REEF_TAG_IDS = {21, 22, 17, 18, 19, 20};
          public static final int[] RED_ALLIANCE_REEF_TAG_IDS = {10, 9, 8, 7, 6, 11};

          public static class L1 {
            public static double MAX_HEIGHT = 1.50919; // Feet
            public static double SCORE_HEIGHT = 3.3; // Feet
            public static double SCORE_ANGLE = -30; // Degrees
            public static double SCORE_OFFSET = 0.2; // Feet
          }

          public static class L2 {
            public static double MAX_HEIGHT = 2.65748; // Feet
            public static double BRANCH_ANGLE = 35; // Degrees
          }

          public static class L3 {
            public static double MAX_HEIGHT = 3.96982; // Feet
            public static double BRANCH_ANGLE = 35; // Degrees
          }
          
          public static class L4 {
            public static double MAX_HEIGHT = 6.00394; // Feet
            public static double BRANCH_ANGLE = 90; // Degrees
          }

          public static final double BRANCH_LEFT_OFFSET = 0.563040616798; // Feet
          public static final double BRANCH_FOWARD_OFFSET = 0.21141732; // Feet
      }

      public static class AlignConstants {
          public static final Transform2d alignOffset = new Transform2d(new Translation2d(-0.6, 0), new Rotation2d(Units.degreesToRadians(0)));
      }

      public static final double LIFT_ANGLE = 30;
      public static final double CLOSE_DISTANCE = 13; // Feet
  }

  public static class ElevatorConstants {
    public static class Stage1 {
      public static int ID = 3;
      public static double MAX_VELOCITY = 120; // Not sure the unit
      public static double MAX_ACCELERATION = 200; // Not sure the unit
      public static double P = 67;
      public static double I = 0;
      public static double D = 1.1;
      public static double S = 0.05; // Volts
      public static double G = 1.35; // Volts
      public static double V = 17; // Volts/(Meters/Second)
      public static double A = 0.2; // Volts/(Meters/Second^2) 
      public static double MASS = 50; // Pounds
      public static double DRUM_RADIUS = 0.98110236; // Inches
      public static double GEAR_RATIO = 18.5714;
      public static double HARD_MAX_HEIGHT = 2.75; // Feet
      public static double TOLLERANCE = 0.2; // Feet
    }

    public static class Stage2 {
      public static int ID = 2;
      public static double MAX_VELOCITY = 120; // Not sure the unit
      public static double MAX_ACCELERATION = 200; // Not sure the unit
      public static double ABSOLUTE_ENCODER_OFFSET = 0; // Degrees
      public static double P = 66;
      public static double I = 0;
      public static double D = 1.1;
      public static double S = 0.05; // Volts
      public static double G = 0.65; // Volts
      public static double V = 14.52; // Volts/(Meters/Second)
      public static double A = 0.04; // Volts/(Meters/Second^2) 
      public static double MASS = 35; // Pounds
      public static double DRUM_RADIUS = 0.98110236; // Inches
      public static double GEAR_RATIO = 15.7143;
      public static double HARD_MAX_HEIGHT = 1.8; // Feet
      public static double TOLLERANCE = 0.2; // Feet
    }

    public static final double ZERO_HEIGHTS_ABOVE_BASE = 0.520; // Feet
    public static final double[] HEIGHT_SETPOINTS = {0, 2.65748, 3.96982, 6.00394}; // Feet
  }

  public static class ArmConstants {
    public static class Shoulder {
      public static int ID1 = 4;
      //public static int ID2 = 8;
      public static double MAX_VELOCITY = 120; // Not sure the unit
      public static double MAX_ACCELERATION = 200; // Not sure the unit
      public static double P = 0.6;
      public static double I = 0;
      public static double D = 0.05;
      public static double S = 0.05; // Volts
      public static double G = 3.45; // Volts
      public static double V = 0.01; // Volts/(Degrees/Second)
      public static double A = 0.02; // Volts/(Degrees/Second^2) 
      public static double MASS = 10; // Pounds
      public static double MIN_ANGLE = -80; // Degrees
      public static double MAX_ANGLE = 80; // Degrees
      public static double STARTING_ANGLE = 0; // Degrees
      public static double ABSOLUTE_ENCODER_OFFSET = 0; // Degrees
      public static double GEAR_RATIO = 50;
      public static double CENTER_OFFSET_FOWARD = 0.492126; // Feet
      public static double STAGE_OFFSET_UP = 0.958; // Feet
      public static double TOLLERANCE = 3; // Degrees
    }

    public static class Wrist {
      public static int ID = 5;
      public static double MAX_VELOCITY = 120; // Not sure the unit
      public static double MAX_ACCELERATION = 200; // Not sure the unit
      public static double P = 0.5;
      public static double I = 0;
      public static double D = 0.05;
      public static double S = 0.05; // Volts
      public static double G = 3.48; // Volts
      public static double V = 0.01; // Volts/(Degrees/Second)
      public static double A = 0.02; // Volts/(Degrees/Second^2) 
      public static double MASS = 15; // Pounds
      public static double MOI = 0.05; // Moment of innertia jKgMetersSquared
      public static double GEAR_RATIO = 400;
      public static double TOLLERANCE = 2; // Degrees
    }

    public static class Intake {
      public static int ID = 6;

      public static class Simulation {
        public static double WIDTH = 0.7; // Feet
        public static double LENGTH = 0.6; // Feet
        public static boolean ASSUME_START_WITH = true;
      }
    }

    public static class IntakeSensor {
      public static int ID = 0;
    }

    public static double LENGTH = 2; // Feet
    
  }

  public static class DebugConstants {
    public static boolean DEBUG_VISION = true;
    public static boolean DEBUG_ELEVATOR = true;
    public static boolean DEBUG_ARM = true;
    public static boolean DEBUG_WRIST = true;
    public static boolean DEBUG_INTAKE = true;
    public static boolean DEBUG_SIMULATION = true;
    public static boolean ANIMATE_ROBOT = true;
  }
}
