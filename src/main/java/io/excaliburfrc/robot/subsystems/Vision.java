package io.excaliburfrc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class Vision extends SubsystemBase {
  private final PhotonCamera limelight;
  private final DoubleSolenoid lifter;

  private Mode currentMode;
  private CameraPosition currentPosition;
  private final NetworkTableEntry leds =
      NetworkTableInstance.getDefault().getTable("photonvision").getEntry("ledMode");

  private static final int POWER_CELL_PIPELINE = 0;
  private static final int POWER_PORT_PIPELINE = 1;
  private static final int DRIVER_PIPELINE = 2;

  public Vision() {
    limelight = new PhotonCamera("limelight");
    lifter = new DoubleSolenoid(Constants.LL_REV, Constants.LL_FWD);
  }

  public void goTo(Mode mode, CameraPosition position) {
    switchMode(mode);
    pointTo(position);
  }

  public void pointTo(CameraPosition dir) {
    currentPosition = dir;
    lifter.set(dir.pistonState);
  }

  public enum CameraPosition {
    FORWARD(0.57, 0, kForward),
    UP(0.5, Units.degreesToRadians(40), kReverse); // TODO: tune

    private final double height;
    private final double pitch;
    private final DoubleSolenoid.Value pistonState;

    CameraPosition(double height, double pitch, DoubleSolenoid.Value pistonState) {
      this.height = height;
      this.pitch = pitch;
      this.pistonState = pistonState;
    }
  }

  public enum Mode {
    DRIVER,
    BALL(0.1),
    TARGET(2.5);

    private final double targetHeight;

    Mode(double targetHeight) {
      this.targetHeight = targetHeight;
    }

    Mode() {
      this(-1);
    }
  }

  public void switchMode(Mode mode) {
    currentMode = mode;
    switch (mode) {
      case BALL:
        limelight.setDriverMode(false);
        limelight.setPipelineIndex(POWER_CELL_PIPELINE);
        leds.setNumber(1);
        break;
      case DRIVER:
        limelight.setPipelineIndex(DRIVER_PIPELINE);
        leds.setNumber(0);
        limelight.setDriverMode(true);
        break;
      case TARGET:
        limelight.setDriverMode(false);
        limelight.setPipelineIndex(POWER_PORT_PIPELINE);
        leds.setNumber(1);
        break;
    }
  }

  public boolean hasTargets() {
    return limelight.getLatestResult().hasTargets();
  }

  public double getDistance() {
    var latest = limelight.getLatestResult();
    if (currentMode == Mode.DRIVER || !latest.hasTargets()) return -1;
    return PhotonUtils.calculateDistanceToTargetMeters(
        currentPosition.height,
        currentMode.targetHeight,
        currentPosition.pitch,
        Units.degreesToRadians(latest.getBestTarget().getPitch()));
  }

  /** degrees */
  public double getYawOffset() {
    if (!limelight.hasTargets()) return -1;
    return limelight.getLatestResult().getBestTarget().getYaw();
  }

  double lastDist = -1;

  @Override
  public void periodic() {
    SmartDashboard.putNumber("dist", lastDist);
    if (limelight.hasTargets()) lastDist = getDistance();
  }
}
