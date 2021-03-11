package io.excaliburfrc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants;
import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

public class Vision extends SubsystemBase {
  private final PhotonCamera limelight;
  private final DoubleSolenoid lifter;

  private Mode currentMode;
  private CameraPosition currentPosition;

  private static final double POWER_CELL_HEIGHT = 0.1; // should maybe be zero
  private static final int POWER_PORT_PIPELINE = 1;
  private static final int POWER_CELL_PIPELINE = 2;

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
    UP(1.2, 3.4, kReverse); // TODO: tune

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
      this(0);
    }
  }

  public void switchMode(Mode mode) {
    currentMode = mode;
    switch (mode) {
      case BALL:
        limelight.setLED(LEDMode.kDefault);
        limelight.setDriverMode(false);
        limelight.setPipelineIndex(POWER_CELL_PIPELINE);
        break;
      case DRIVER:
        limelight.setLED(LEDMode.kOff);
        limelight.setDriverMode(true);
        break;
      case TARGET:
        limelight.setLED(LEDMode.kDefault);
        limelight.setDriverMode(false);
        limelight.setPipelineIndex(POWER_PORT_PIPELINE);
        break;
    }
  }

  public boolean hasTargets() {
    return limelight.getLatestResult().hasTargets();
  }

  public double getDistance() {
    if (currentMode == Mode.DRIVER) return 0;
    return PhotonUtils.calculateDistanceToTargetMeters(
        currentPosition.height,
        currentMode.targetHeight,
        currentPosition.pitch,
        Units.degreesToRadians(limelight.getLatestResult().getBestTarget().getPitch()));
  }

  public double getXOffset() {
    if (hasTargets()) return 0;
    return limelight.getLatestResult().getBestTarget().getYaw();
  }

 double lastDist = -1;
  @Override
  public void periodic() {

    SmartDashboard.putNumber("dist", lastDist);
    if (limelight.hasTargets()) lastDist = getDistance();
  }
}
