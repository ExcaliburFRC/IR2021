package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.TestUtil.DELTA;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import io.excaliburfrc.TestUtil;
import io.excaliburfrc.robot.Constants.TransporterConstants;
import io.excaliburfrc.robot.subsystems.Transporter.Mode;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class TransporterTest {
  SimDouble simProx;
  Transporter transporter;
  SimDouble simLoading;
  SimDouble simFlicker;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 1);
    transporter = new Transporter();
    simProx = new SimDeviceSim("REV Color Sensor V3[0,82]").getDouble("Proximity");
    simFlicker = new SimDeviceSim("CANMotor:Victor SPX[33]").getDouble("percentOutput");
    simLoading = new SimDeviceSim("CANMotor:Victor SPX[32]").getDouble("percentOutput");
    DriverStationSim.setEnabled(true);
    HAL.simPeriodicBefore();
  }

  @AfterEach
  void teardown() throws Exception {
    transporter.close();
    SimDeviceSim.resetData();
  }

  @Test
  void verifyLimit() {
    simProx.set(1.7 * TransporterConstants.LIMIT);
    assertTrue(transporter.isBallReady());
  }

  @Test
  void stopOnSensor() {
    simProx.set(1.7 * TransporterConstants.LIMIT);
    assertTrue(transporter.isBallReady());
    transporter.activate(Mode.IN);
    TestUtil.stepCTRE();
    assertAll(
        () -> assertEquals(0, simFlicker.get(), TestUtil.DELTA),
        () -> assertEquals(0, simLoading.get(), TestUtil.DELTA));
  }

  @ParameterizedTest
  @EnumSource(Transporter.Mode.class)
  void setsValues(Transporter.Mode mode) {
    simProx.set(0.3 * TransporterConstants.LIMIT);
    assertFalse(transporter.isBallReady());
    transporter.activate(mode);
    TestUtil.stepCTRE();
    assertAll(
        () -> assertEquals(mode.flicker, simFlicker.get(), DELTA),
        () -> assertEquals(mode.loading, simLoading.get(), DELTA));
  }

  @Test
  void test() {
    transporter.activate(Mode.SHOOT);
    TestUtil.stepCTRE();
    assertAll(
        () -> assertEquals(Mode.SHOOT.flicker, simFlicker.get(), TestUtil.DELTA),
        () -> assertEquals(Mode.SHOOT.loading, simLoading.get(), TestUtil.DELTA));
  }
}
