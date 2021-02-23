package io.excaliburfrc.lib;

import com.revrobotics.*;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpiutil.math.MathUtil;

public class CANPIDControllerSim extends CANPIDController implements Runnable {
  private final SimSparkMax device;
  private SimDeviceSim simDevice;
  private SimDouble appliedOutput, input;
  private PIDController controller = new PIDController(0, 0, 0);
  private ControlType refType = ControlType.kDutyCycle;
  private double ref = 0.0;
  private double kF = 0.0;
  private boolean alt = false;

  public CANPIDControllerSim(SimSparkMax device, SimDeviceSim simDevice) {
    //noinspection removal
    super(device);
    this.device = device;
    this.simDevice = simDevice;
    this.appliedOutput = simDevice.getDouble(ctrlToKey(ControlType.kDutyCycle, false));
  }

  @Override
  public CANError setReference(double value, ControlType ctrl) {
    if(ctrl == ControlType.kDutyCycle)       DriverStation.reportError("Caller:", true);

    this.refType = ctrl;
    this.ref = value;
    input = simDevice.getDouble(ctrlToKey(ctrl, alt));
    return CANError.kOk;
  }

  @Override
  public void run() {
    if (refType == ControlType.kDutyCycle) {
      appliedOutput.set(ref);
//      System.out.println("appliedOutput.get() = " + appliedOutput.get());
      return;
    }
    System.out.println("refType = " + refType);
    System.out.println("ref = " + ref);
    if (refType == ControlType.kVelocity) {
      System.out.println("input.get() = " + input.get());
      var calculate = controller.calculate(input.get(), ref);
      System.out.println("calculate = " + calculate);
      var ff = kF * ref;
      System.out.println("ff = " + ff);
      appliedOutput.set(MathUtil.clamp(calculate + ff, -1, 1));
      System.out.println("appliedOutput.get() = " + appliedOutput.get());
      return;
    }
    DriverStation.reportError("Control Type " + refType + "isn't implemented yet.", false);
  }

  public static String ctrlToKey(ControlType ctrl, boolean alt) {
    switch (ctrl) {
      case kDutyCycle:
        return "Applied Output";
      case kVelocity:
        return (alt) ? "Alt Encoder Velocity" : "Velocity";
      case kPosition:
        return (alt) ? "Alt Encoder Position" : "Position";
      case kCurrent:
        return "Current";
      case kVoltage:
        return "Voltage";
      default:
        break;
    }
    throw new UnsupportedOperationException("not implemented yet:" + ctrl);
  }

  @Override
  public CANError setP(double gain) {
    controller.setP(gain);
    return CANError.kOk;
  }

  @Override
  public CANError setI(double gain) {
    controller.setI(gain);
    return CANError.kOk;
  }

  @Override
  public CANError setD(double gain) {
    controller.setD(gain);
    return CANError.kOk;
  }

  @Override
  public CANError setFF(double gain) {
    this.kF = gain;
    return CANError.kOk;
  }

  @Override
  public CANError setFeedbackDevice(CANSensor sensor) {
    alt = true; // TODO: this **will** break!!!!
    return CANError.kOk;
  }
}
