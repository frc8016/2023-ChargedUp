// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class EndEffectorConstants {
    public static final int INTAKE_ID = 7;

    public static final int SOLENOID_FORWARD_CHANNEL = 0;
    public static final int SOLENOID_REVERSE_CHANNEL = 1;

    public static final double INTAKE_SPEED = .1;
  }

  public static class DrivetrainConstants {
    public static final int FRONT_LEFT_MOTOR_ID = 1;
    public static final int BACK_LEFT_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_MOTOR_ID = 3;
    public static final int BACK_RIGHT_MOTOR_ID = 4;
  }

  public static class ArmConstants {
    public static final int RELATIVE_ENCODER_A = 9;
    public static final int RELATIVE_ENCODER_B = 8;
    public static final int ABSOLUTE_ENCODER_PORT = 0;

    public static final int LEFT_SHOULDER_MOTOR_ID = 6;
    public static final int RIGHT_SHOULDER_MOTOR_ID = 5;

    public static final double ks = 0.20394;
    public static final double kg = 0.44787;
    public static final double kv = 2.1809;
    public static final double ka = 0.38427;

    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0;

    public static final double kMaxVelocityRadPerSecond = Math.PI / 2;
    public static final double kMaxAccelerationRadPerSecondSquared = 2.36;

    // kArmOffsetRadians must equal - Math.PI/2, since the arm rests 90 degrees from
    // horizontal; per
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ArmFeedforward.html
    public static final double kArmOffsetRadians = 4.76;
    public static final double kArmLengthMeters = 0.8461756;
    public static final double kFulcrumHeightFromFloorMeters = 1.016032766;
    public static final double kRelativeEncoderRadiansPerPulse = 2 * Math.PI / 1024;

    enum ArmPosition {
      kHybrid(kFulcrumHeightFromFloorMeters - kArmLengthMeters + .1),
      kConeLevel2(.87),
      kCubeLevel2(.6),
      kIndex(kFulcrumHeightFromFloorMeters - kArmLengthMeters + .1);

      public final double value;

      // Computes necessary arm angle given distance from floor (in meters)
      ArmPosition(double value) {
        this.value = Math.asin((value - kFulcrumHeightFromFloorMeters) / kArmLengthMeters);
      }
    }
  }
}
