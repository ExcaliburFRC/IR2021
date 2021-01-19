package io.excaliburfrc.robot;

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
    public static final int FORWARD_CHANNEL = 3;
    public static final int REVERSE_CHANNEL = 2;
  }

  public static final class DriveConstants {
    public static final int RIGHT_LEADER_ID = 11;
    public static final int RIGHT_FOLLOWER_ID = 12;
    public static final int LEFT_LEADER_ID = 13;
    public static final int LEFT_FOLLOWER_ID = 14;
    public static final int TPS = 2046;
    public static final int TRACK_WIDTH = 60;
  }
  // constants should be declared as `public static final double`
  // constant names should be in `SCREAMING_SNAKE_CASE` or `kUpperCamelCase`

  public static final class ShooterConstants {
    public static final int SHOOTER_ID = 13;
  }public static final class TransporterConstants {
    public static final int FLICKER_ID = 33;
    public static final int LOADING_ID = 32;
    public static final int LIMIT = 20;
  }
}
