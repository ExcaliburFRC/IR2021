package io.excaliburfrc.lib;

import com.revrobotics.*;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.hal.simulation.SimulatorJNI.SimPeriodicBeforeCallback;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimSparkMax extends CANSparkMax {
  private SimPeriodicBeforeCallback callback;
  private final SimDouble simappliedOutput;
  private final SimDeviceSim simDevice;
  private CANPIDController controller;

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
      simDevice = new SimDeviceSim("SPARK MAX [" + deviceID + "]");
      simappliedOutput = simDevice.getDouble("Applied Output");
    } else {
      simDevice = null;
      simappliedOutput = null;
    }
  }

  @Override
  public void set(double speed) {
    if (RobotBase.isSimulation()) {
      getPIDController().setReference(speed, ControlType.kDutyCycle);
    }
//    super.set(speed);
  }

  @Override
  public CANPIDController getPIDController() {
    if (controller == null) {
      if (RobotBase.isSimulation()) {
        controller = new CANPIDControllerSim(this, simDevice);
        callback = SimulatorJNI.registerSimPeriodicBeforeCallback((Runnable) controller);
      } else {
        controller = super.getPIDController();
      }
    }
    return controller;
  }

  @Override
  public void close() {
    if (callback != null) {
      callback.close();
    }
    super.close();
  }
}
