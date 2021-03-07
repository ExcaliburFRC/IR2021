package io.excaliburfrc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.vision.Limelight;
import io.excaliburfrc.lib.vision.Limelight.DataKey;
import io.excaliburfrc.lib.vision.Limelight.SettingsKey;
import io.excaliburfrc.robot.Constants;

public class Vision extends SubsystemBase {
  private final Limelight limelight;
  private final DoubleSolenoid piston;

  private static final int POWER_PORT_PIPELINE = 1;
  private static final int POWER_CELL_PIPELINE = 2;

  public Vision() {
    limelight = new Limelight();
    piston = new DoubleSolenoid(Constants.LIMELIGHT_FWD, Constants.LIMELIGHT_REV);
  }

  public void raise() {
    piston.set(DoubleSolenoid.Value.kReverse);
    limelight.setData(SettingsKey.ActivePipeline, POWER_PORT_PIPELINE);
  }

  public void lower() {
    piston.set(DoubleSolenoid.Value.kForward);
    limelight.setData(SettingsKey.ActivePipeline, POWER_CELL_PIPELINE);
  }

  public double getDistance() {
    //  TODO : calibrate [regression]
    return -1;
  }

  public double getXOffset() {
    return limelight.getData(DataKey.XOffset);
  }

}
