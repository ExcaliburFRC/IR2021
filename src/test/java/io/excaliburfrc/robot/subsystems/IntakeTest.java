package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.TestUtil.DELTA;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.*;
import io.excaliburfrc.TestUtil;
import io.excaliburfrc.robot.Constants.IntakeConstants;
import io.excaliburfrc.robot.subsystems.Intake.Mode;
import java.util.Arrays;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.function.Executable;

class IntakeTest {
  static Intake intake;
  static SimDouble simAppliedOutput;
  static PCMSim simPcm;

  @BeforeAll
  static void setup() {
    assert HAL.initialize(500, 0);
    intake = new Intake();
    simAppliedOutput =
        new SimDeviceSim("CANMotor:Victor SPX[" + IntakeConstants.INTAKE_MOTOR_ID + "]")
            .getDouble("percentOutput");
    simPcm = new PCMSim();
    DriverStationSim.setEnabled(true);
    HAL.simPeriodicBefore();
  }

  @Test
  void doesNotWorkWhenClosed() {
    intake.raise();
    intake.activate(Mode.IN);
    TestUtil.stepCTRE();
    assertEquals(0, simAppliedOutput.get(), DELTA);
    intake.activate(Mode.OUT);
    TestUtil.stepCTRE();
    assertEquals(0, simAppliedOutput.get(), DELTA);
  }

  @Test
  void setsPwm() {
    intake.lower();
    Executable[] array =
        Arrays.stream(Mode.values())
            .<Executable>map(
                mode ->
                    () -> {
                      HAL.simPeriodicBefore();
                      intake.activate(mode);
                      TestUtil.stepCTRE();
                      assertEquals(mode.speed, simAppliedOutput.get(), DELTA);
                    })
            .toArray(Executable[]::new);
    assertAll(array);
  }

  @Test
  void setsPcm() {
    intake.lower();
    assertAll(
        "Lowering tests:",
        () -> assertTrue(simPcm.getSolenoidOutput(IntakeConstants.FORWARD_CHANNEL)),
        () -> assertFalse(simPcm.getSolenoidOutput(IntakeConstants.REVERSE_CHANNEL)));
    intake.raise();
    assertAll(
        "Raising tests:",
        () -> assertFalse(simPcm.getSolenoidOutput(IntakeConstants.FORWARD_CHANNEL)),
        () -> assertTrue(simPcm.getSolenoidOutput(IntakeConstants.REVERSE_CHANNEL)),
        () -> assertEquals(0, simAppliedOutput.get(), DELTA));
  }

  @Test
  void test() {
    intake.lower();
    assertTrue(intake.isOpen());
    intake.activate(Mode.IN);
    TestUtil.stepCTRE();
    assertEquals(Mode.IN.speed, simAppliedOutput.get(), DELTA);
  }
}
