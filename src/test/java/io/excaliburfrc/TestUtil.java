package io.excaliburfrc;

import edu.wpi.first.hal.HAL;

public class TestUtil {
  public static final double DELTA = 1e-2;

  public static void stepCTRE() {
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    HAL.simPeriodicBefore();
  }
}
