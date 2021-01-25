package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.TransporterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transporter extends SubsystemBase {
  private VictorSPX flicker, loading;
  private ColorSensorV3 ballDetector;

  public Transporter() {
    flicker = new VictorSPX(FLICKER_ID);
    loading = new VictorSPX(LOADING_ID);
    ballDetector = new ColorSensorV3(I2C.Port.kOnboard);
  }

  public enum Mode {
    IN(0.6),
    OUT(-0.4),
    OFF(0);

    Mode(double i) {
      speed = i;
    }

    public final double speed;
  }

  public void setFlicker(Mode speed) {
    flicker.set(ControlMode.PercentOutput, speed.speed);
  }

  public void setLoading(Mode speed) {
    loading.set(ControlMode.PercentOutput, speed.speed);
  }

  public boolean isBallReady() {
    return ballDetector.getProximity() < LIMIT;
  }
}
