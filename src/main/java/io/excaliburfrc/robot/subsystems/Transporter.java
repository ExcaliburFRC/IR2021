package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.TransporterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transporter extends SubsystemBase {
  private VictorSPX flicker, loading;

  public Transporter() {
    flicker = new VictorSPX(FLICKER_ID);
    loading = new VictorSPX(LOADING_ID);
  }

  public void setFlickerspeed(double speed) {
    flicker.set(ControlMode.PercentOutput, speed);
  }

  public void setLoading(double speed) {
    loading.set(ControlMode.PercentOutput, speed);
  }
}
