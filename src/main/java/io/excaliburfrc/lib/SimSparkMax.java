package io.excaliburfrc.lib;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.hal.simulation.SimulatorJNI.SimPeriodicBeforeCallback;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimSparkMax extends CANSparkMax {
  private SimPeriodicBeforeCallback callback;
  private final SimDeviceSim simDevice;

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
    } else {
      simDevice = null;
    }
  }

  @Override
  public void set(double speed) {
    getPIDController().setReference(speed, ControlType.kDutyCycle);
  }

  @Override
  public void close() {
    if (callback != null) {
      callback.close();
    }
    super.close();
    CANSparkMaxJNI.c_SparkMax_Destroy(m_sparkMax);
  }

  @Override
  public CANPIDController getPIDController() {
    if (RobotBase.isSimulation()) {
      var controller = new CANPIDControllerSim(this, simDevice);
      callback = SimulatorJNI.registerSimPeriodicBeforeCallback(controller);
      return controller;
    } else return super.getPIDController();
  }
}
