package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.TestConstants.DELTA;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import io.excaliburfrc.robot.Constants.TransporterConstants;
import io.excaliburfrc.robot.subsystems.Transporter.Mode;
import org.junit.jupiter.api.*;

class TransporterTest {
  static SimDouble simProx;
  static Transporter transporter;
  static SimDouble simLoading;
  static SimDouble simFlicker;

  @BeforeAll
  static void setup() {
    assert HAL.initialize(500, 1);
    transporter = new Transporter();
    simProx = new SimDeviceSim("REV Color Sensor V3[0,82]").getDouble("Proximity");
    simFlicker = new SimDeviceSim("CANMotor:Victor SPX[33]").getDouble("percentOutput");
    simLoading = new SimDeviceSim("CANMotor:Victor SPX[32]").getDouble("percentOutput");
  }

  @Test
  void verifyLimit() {
    simProx.set(0.7 * TransporterConstants.LIMIT);
    assertTrue(transporter.isBallReady());
  }

  @Test
  void stopOnSensor() {
    simProx.set(0.7 * TransporterConstants.LIMIT);
    transporter.activate(Mode.IN);
    assertAll(
        () -> assertEquals(0, simFlicker.get(), DELTA),
        () -> assertEquals(0, simLoading.get(), DELTA));
  }
}
