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
    public static final int FORWARD_CHANNEL = 3;
    public static final int REVERSE_CHANNEL = 2;
  }

  public static final class DriveConstants {
    public static final int RIGHT_LEADER_ID = 11;
    public static final int RIGHT_FOLLOWER_ID = 12;
    public static final int LEFT_LEADER_ID = 13;
    public static final int LEFT_FOLLOWER_ID = 14;
    public static final int CPR = 2046;
    public static final double TRACK_WIDTH = 0.60;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(6);
    public static final double GEARING = 10.25;
    public static final double kV_lin = 1.98, kA_lin = 0.2, kV_ang = 1.5, kA_ang = 0.3;
    public static final double PULSE_TO_METER = (CPR * GEARING) / (WHEEL_RADIUS  * Math.PI);
  }
  // constants should be declared as `public static final double`
  // constant names should be in `SCREAMING_SNAKE_CASE` or `kUpperCamelCase`

  public static final class ShooterConstants {
    public static final int SHOOTER_ID = 41;
    public static final double TOLERANCE = 100; // TODO: tune
    public static final double GEARING = 3; // TODO: tune
    public static final double kV = 1.2, kA = 0.3; // TODO: tune
    public static final double kP = 0.002, kF = 0.11;
    //    1.98, 0.2, 1.5, 0.3
  }

  public static final class TransporterConstants {
    public static final int FLICKER_ID = 33;
    public static final int LOADING_ID = 32;
    public static final int LIMIT = 20;
  }

  public static final class ClimberConstants {
    public static final int LEADER_ID = 51;
    public static final int FOLLOWER_ID = 50;
    public static final int HANGER_REV = 2;
    public static final int HANGER_FWD = 3;
    public static final double UP_SPEED = 0.6;
    public static final double DOWN_SPEED = -0.4;
  }
}
