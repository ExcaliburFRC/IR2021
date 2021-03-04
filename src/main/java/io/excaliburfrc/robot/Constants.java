package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // add a inner class `public static final class` for each subsystem.
  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 60;
    public static final int FORWARD_CHANNEL = 1;
    public static final int REVERSE_CHANNEL = 0;
  }

  public static final class DriveConstants {
    public static final int RIGHT_LEADER_ID = 11;
    public static final int RIGHT_FOLLOWER_ID = 12;
    public static final int LEFT_LEADER_ID = 13;
    public static final int LEFT_FOLLOWER_ID = 14;
    public static final double TRACK_WIDTH = 0.7047364141920852;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(6);
    public static final double GEARING = 0.09425070688030161;
    public static final double kV_lin = 2.66, kA_lin = 0.433, kV_ang = 2.76, kA_ang = -0.236;
    public static final double PULSE_TO_METER =
          GEARING * WHEEL_RADIUS * Math.PI;
//        1 / 23.5; // (CPR * GEARING) / (WHEEL_RADIUS * Math.PI);
    public static final double kP = 0.0398;
    public static final double kS = 0.129, kS_ang = 0.285; // TODO: tune
  }
  // constants should be declared as `public static final double`
  // constant names should be in `SCREAMING_SNAKE_CASE` or `kUpperCamelCase`

  public static final class ShooterConstants {
    public static final int SHOOTER_ID = 41;
    public static final double TOLERANCE = 100; // TODO: tune
    public static final double GEARING = 2.0;
    public static final double kS = 0.22, kV = 0.258, kA = 0.0972;
    public static final double kP = 0.046;
  }

  public static final class TransporterConstants {
    public static final int FLICKER_ID = 33;
    public static final int LOADING_ID = 32;
    public static final int LIMIT = 100;
  }

  public static final class ClimberConstants {
    public static final int LEADER_ID = 51;
    public static final int FOLLOWER_ID = 50;
    public static final int HANGER_REV = 2;
    public static final int HANGER_FWD = 3;
    public static final double UP_SPEED = 0.6;
    public static final double DOWN_SPEED = -0.4;
  }

  public static final int LED_PORT = 4;
}
