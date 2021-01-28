package io.excaliburfrc.lib;

import com.revrobotics.*;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimSparkMax extends CANSparkMax {
  private final SimDouble simappliedOutput;
  private SimDeviceSim simDevice;

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
      simappliedOutput = simDevice.getDouble(CanUtil.ctrlToKey(ControlType.kDutyCycle));
    } else simappliedOutput = null;
  }

  @Override
  public void set(double speed) {
    if (RobotBase.isSimulation()) {
      simappliedOutput.set(speed);
    }
    super.set(speed);
  }

  @Override
  public CANPIDController getPIDController() {
    return new CANPIDControllerSim(this, simDevice);
  }

  private static final class OnboardControl {
    private final PIDController controller;
    private final SimDouble input, output;

    public OnboardControl(CANPIDController controller, SimDouble input, SimDouble output) {
      this.controller = new PIDController(controller.getP(), controller.getI(), controller.getD());
      this.input = input;
      this.output = output;
    }

    private static final Notifier notifier = new Notifier(OnboardControl::run);
    private static boolean isRunning = false;
    private static final double PERIOD = 0.02;

    static synchronized void start() {
      if (isRunning) {
        return;
      }
      notifier.startPeriodic(PERIOD);
      isRunning = true;
    }

    static synchronized void stop() {
      if (!isRunning) {
        return;
      }
      notifier.stop();
      isRunning = false;
    }

    private static void run() {}

    private void calc() {}
  }
}
