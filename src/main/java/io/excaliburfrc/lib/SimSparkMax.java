package io.excaliburfrc.lib;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimSparkMax extends CANSparkMax {
  private final SimDouble simappliedOutput;

  /**
   * Create a new SPARK MAX Controller
   *
   * @param deviceID The device ID.
   * @param type The motor type connected to the controller. Brushless motors must be connected to
   *     their matching color and the hall sensor plugged in. Brushed motors must be connected to
   *     the Red and
   */
  public SimSparkMax(int deviceID, MotorType type) {
    super(deviceID, type);
    if (RobotBase.isSimulation()) {
      simappliedOutput =
          new SimDeviceSim("SPARK MAX [" + deviceID + "]").getDouble("Applied Output");
    } else simappliedOutput = null;
  }

  @Override
  public void set(double speed) {
    if (RobotBase.isSimulation()) {
      simappliedOutput.set(speed);
    }
    super.set(speed);
  }
}
