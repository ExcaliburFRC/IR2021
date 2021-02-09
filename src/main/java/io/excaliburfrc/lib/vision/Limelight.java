package io.excaliburfrc.lib.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  private final NetworkTable table;

  /** Create a Limelight with the default table name ("limelight"). */
  public Limelight() {
    this("limelight");
  }

  public Limelight(String tablename) {
    table = NetworkTableInstance.getDefault().getTable(tablename);
  }

  /**
   * Set config data to the limelight.
   *
   * <p>Use the specific functions for operations.
   *
   * @see SettingsKey
   */
  public void setData(SettingsKey key, int value) {
    table.getEntry(key.key).setNumber(value);
  }

  /**
   * Read data from the limelight.
   *
   * @see DataKey
   */
  public double getData(DataKey key) {
    return table.getEntry(key.key).getDouble(0);
  }

  public enum DataKey {
    /** Whether the limelight has any valid targets (0 or 1). */
    HasValidTargets("tv"),
    /** Horizontal Offset From Crosshair To Target. (LL1: [-27, 27] deg | LL2: [-29.8, 29.8] deg) */
    XOffset("tx"),
    /**
     * Vertical Offset From Crosshair To Target. (LL1: [-20.5, 20.5] deg | LL2: [-24.85, 24.85] deg)
     */
    YOffset("ty"),
    /** Target Area. (0% of image to 100% of image) */
    TargetArea("ta"),
    /** Skew or rotation (-90 degrees to 0 degrees) */
    Skew("ts"),
    /** The pipeline’s latency contribution (ms). Add at least 11ms for image capture latency. */
    PipelineLatency("tl"),
    /** Sidelength of shortest side of the fitted bounding box (pixels). */
    LengthShort("tshort"),
    /** Sidelength of shortest side of the fitted bounding box (pixels). */
    LengthLong("tlong"),
    /** Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
    LengthHorizontal("thor"),
    /** Vertical sidelength of the rough bounding box (0 - 320 pixels) */
    LengthVertical("tvert"),
    /** True active pipeline index of the camera (0 .. 9). */
    ActivePipelineIndex("getpipe");

    DataKey(String s) {
      this.key = s;
    }

    public final String key;
  }

  public enum SettingsKey {
    /**
     * Sets the limelight’s LED state 0 use the LED Mode set in the current pipeline 1 force off 2
     * force blink 3 force on
     */
    LedMode("ledMode"),
    /**
     * Sets limelight’s operation mode 0 Vision processor 1 Driver Camera (Increases exposure,
     * disables vision processing)
     */
    CameraMode("camMode"),
    /** Set active pipeline (0..9). */
    ActivePipeline("pipeline"),
    /**
     * Sets limelight’s streaming mode. 0 Standard - Side-by-side streams if a webcam is attached to
     * Limelight 1 PiP Main - The secondary camera stream is placed in the lower-right corner of the
     * primary camera stream 2 PiP Secondary - The primary camera stream is placed in the
     * lower-right corner of the secondary camera stream
     */
    StreamMode("stream"),
    /**
     * Allows users to take snapshots during a match 0 Stop taking snapshots 1 Take two snapshots
     * per second
     */
    Snapshot("snapshot");

    SettingsKey(String s) {
      this.key = s;
    }

    public final String key;
  }

  public enum CameraMode {
    VISION(0),
    DRIVER(1);

    CameraMode(int i) {
      key = i;
    }

    public final int key;
  }

  public void setCameraMode(CameraMode mode) {
    setData(SettingsKey.CameraMode, mode.key);
  }

  public enum LedMode {
    PIPE_DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    LedMode(int i) {
      key = i;
    }

    public final int key;
  }

  public void setLedMode(LedMode mode) {
    setData(SettingsKey.LedMode, mode.key);
  }

  public boolean hasValidTargets() {
    return getData(DataKey.HasValidTargets) == 1;
  }
}
