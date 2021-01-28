package io.excaliburfrc.lib;

import com.revrobotics.ControlType;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class CANEncoderSim {
  private final SimDouble position, velocity;

  public CANEncoderSim(boolean alt, int deviceID) {
    var device = new SimDeviceSim("SPARK MAX [" + deviceID + "]");
    position = device.getDouble(CanUtil.ctrlToKey(ControlType.kPosition, alt));
    velocity = device.getDouble(CanUtil.ctrlToKey(ControlType.kVelocity, alt));
  }

  public void setVelocity(double vel) {
    velocity.set(vel);
  }

  public void setPosition(double pos) {
    position.set(pos);
  }
}
