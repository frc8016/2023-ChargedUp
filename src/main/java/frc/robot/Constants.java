// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// Please note that all meters, unless otherwise specified, will be used for ALL distance
// measurements. Kilograms, unless otherwise specified, will be used for ALL mass measurements.
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int JOYSTICK_PORT = 1;
    public static final int JOYSTICK_Y_AXIS = 1;
    public static final int JOYSTICK_X_AXIS = 0;
  }

  public static class AutonConstants {
    public static final double SCORE_DELAY = 2;
    public static final double RAISE_DELAY = 3;
    public static final double LOWER_DELAY = 0;

    public static final double CUBE_HIGH_TIER_SPEED = -1;
    public static final double TAXI_SPEED = .5;

    public static final double TAXI_DISTANCE = 130;

    public static final double kPDrive = 0;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;
  }

  public static class EndEffectorConstants {
    public static final int INTAKE_ID = 7;

    public static final int MOTOR_CURRENT_LIMIT = 15; // Amps

    public static final int SOLENOID_FORWARD_CHANNEL = 0;
    public static final int SOLENOID_REVERSE_CHANNEL = 1;

    public static final double INTAKE_SPEED = .1;
  }

  public static class DrivetrainConstants {
    public static final int FRONT_LEFT_MOTOR_ID = 1;
    public static final int BACK_LEFT_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_MOTOR_ID = 3;
    public static final int BACK_RIGHT_MOTOR_ID = 4;

    public static final int MOTOR_CURRENT_LIMIT = 35; // Amps
    public static final double TRACK_WIDTH_METERS = 0;

    public static final int LEFT_DRIVE_ENCODER_ID = 0;
    public static final int RIGHT_DRIVE_ENCODER_ID = 0;

    public static final int PIGEON_ID = 0;

    public static final Matrix<N3, N1> STATE_STD_DEVS =
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
  }

  public static class ArmConstants {
    public static final int RELATIVE_ENCODER_A = 9;
    public static final int RELATIVE_ENCODER_B = 8;
    public static final int ABSOLUTE_ENCODER_PORT = 0;

    public static final int LEFT_SHOULDER_MOTOR_ID = 6;
    public static final int RIGHT_SHOULDER_MOTOR_ID = 5;

    public static final int MOTOR_CURRENT_LIMIT = 20; // Amps

    public static final double ks = 0.20394;
    public static final double kg = 0.44787;
    public static final double kv = 2.1809;
    public static final double ka = 0.38427;

    public static final double kp = 4;
    public static final double ki = 0;
    public static final double kd = 0;

    public static final double kMaxVelocityRadPerSecond = Math.PI / 4;
    public static final double kMaxAccelerationRadPerSecondSquared = 1;

    // kArmOffsetRadians must equal - Math.PI/2, since the arm rests 90 degrees from
    // horizontal; per
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ArmFeedforward.html
    public static final double kArmOffsetRadians = 4.76;
    public static final double kArmLengthMeters = 0.8461756;
    public static final double kFulcrumHeightFromFloorMeters = 1.016032766;
    public static final double kRelativeEncoderRadiansPerPulse = 2 * Math.PI / 1024;

    // non-computed positions; this will be depreciated following softlimits on arm rotation.

    public static final double FLOOR_INTAKE_POSE = -Math.PI / 2 + Math.PI / 30;
    public static final double CUBE_LAUNCH_POSE = Math.PI / 24;
    public static final double INDEX_POSE = -Math.PI / 2 + Math.PI / 6;

    enum ArmPosition {
      kHybrid(kFulcrumHeightFromFloorMeters - kArmLengthMeters + .1),
      kConeLevel2(.87),
      kCubeLevel2(.6),
      kIndex(kFulcrumHeightFromFloorMeters - kArmLengthMeters + .2);

      public final double value;

      // Computes necessary arm angle given distance from floor (in meters)
      ArmPosition(double value) {
        this.value = Math.asin((value - kFulcrumHeightFromFloorMeters) / kArmLengthMeters);
      }
    }
  }

  public static class LEDConstants {
    public static final int FRONT_LED_LENGTH = 21;
    public static final int BACK_LED_LENGTH = 9;
    public static final int TOTAL_LED_LENGTH = 2 * FRONT_LED_LENGTH + 2 * BACK_LED_LENGTH;
    public static final int LED_PORT = 0;
    public static final int[] TEAM_GREEN = {127, 255, 0};
    public static final int[] TEAM_BLUE = {0, 0, 255};
    public static final int[] TEAM_CYAN = {47, 88, 255};
    public static final int[] CONE_YELLOW = {255, 110, 0};
    public static final int[] CUBE_PURPLE = {200, 0, 255};
  }
}
