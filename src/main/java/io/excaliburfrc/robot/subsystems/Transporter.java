package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.TransporterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transporter extends SubsystemBase {
  private final WPI_VictorSPX flicker, loading;
  private final ColorSensorV3 ballDetector;

  public Transporter() {
    flicker = new WPI_VictorSPX(FLICKER_ID);
    loading = new WPI_VictorSPX(LOADING_ID);
    loading.setInverted(true);
    ballDetector = new ColorSensorV3(I2C.Port.kOnboard);
  }

  public void close() {
    flicker.DestroyObject();
    loading.DestroyObject();
    I2CJNI.i2CClose(Port.kOnboard.value);
  }

  public enum Mode {
    // fixme - change values if needed
    // (flickers, loading)
    SHOOT(0.5, 0.5),
    IN(0.5, 0.5),
    OUT(-0.4, -0.3),
    OFF(0, 0);

    Mode(double fl, double ld) {
      flicker = fl;
      loading = ld;
    }

    public final double flicker, loading;
  }

  public void activate(Mode mode) {
    // if intaking, don't let the ball pass the sensor
    if (mode == Mode.IN && isBallReady()) {
      mode = Mode.OFF;
    }
    flicker.set(ControlMode.PercentOutput, mode.flicker);
    loading.set(ControlMode.PercentOutput, mode.loading);
  }

  public void stop() {
    activate(Mode.OFF);
  }

  public boolean isBallReady() {
    return ballDetector.getProximity() > LIMIT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("prox", ballDetector.getProximity());
    SmartDashboard.putBoolean("isReady", isBallReady());
  }
}
