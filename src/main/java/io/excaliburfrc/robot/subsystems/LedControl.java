package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.LED_PORT;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedControl extends SubsystemBase {
  private final PWMSpeedController leds = new PWMSparkMax(LED_PORT);

  public void setLEDs(LedMode mode) {
    leds.set(mode.val);
  }

  public enum LedMode {
    ;

    final double val;

    LedMode(double val) {
      this.val = val;
    }
  }
}
