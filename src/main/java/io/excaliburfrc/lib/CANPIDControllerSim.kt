@file:JvmName("CanUtil")

package io.excaliburfrc.lib

import com.revrobotics.*
import edu.wpi.first.hal.SimDouble
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import edu.wpi.first.wpilibj2.command.RunCommand

internal class CANPIDControllerSim(device: CANSparkMax, private val simDevice: SimDeviceSim) :
    CANPIDController(device) {
  init {
    instances += this
  }

  private var controller = PIDController(0.0, 0.0, 0.0)
  private var refType: SimDouble? = null
  private var ref = 0.0
  private var feedforward = 0.0
  private var alt = false
  private val appliedOutput: SimDouble = simDevice.getDouble(ctrlToKey(ControlType.kDutyCycle))

  override fun setReference(value: Double, ctrl: ControlType): CANError {
    ref = value
    refType = simDevice.getDouble(ctrlToKey(ctrl, alt))
    calculate()
    if (!command.isScheduled) command.schedule()
    return super.setReference(value, ctrl)
  }

  override fun setP(gain: Double): CANError {
    controller.p = gain
    return CANError.kOk
  }

  override fun setI(gain: Double): CANError {
    controller.i = gain
    return CANError.kOk
  }

  override fun setD(gain: Double): CANError {
    controller.d = gain
    return CANError.kOk
  }

  override fun setFF(gain: Double): CANError {
    feedforward = gain
    return CANError.kOk
  }

  override fun getP(): Double = controller.p

  override fun getI(): Double = controller.i

  override fun getD(): Double = controller.d

  override fun getFF(): Double = feedforward

  override fun setFeedbackDevice(sensor: CANSensor?): CANError {
    alt = true // this will break sometime
    return super.setFeedbackDevice(sensor)
  }

  fun calculate() {
    val _refType = refType
    if (_refType != null) {
      appliedOutput.set(feedforward + controller.calculate(_refType.get(), ref))
    } else {
      appliedOutput.set(ref)
    }
  }

  companion object {
    private val instances = HashSet<CANPIDControllerSim>()
    private val command = RunCommand({ instances.forEach { it.calculate() } })
  }
}

@JvmOverloads
fun ctrlToKey(ctrl: ControlType, alt: Boolean = false): String {
  return when (ctrl) {
    ControlType.kDutyCycle -> "Applied Output"
    ControlType.kVelocity -> if (alt) "Alt Encoder Velocity" else "Velocity"
    ControlType.kPosition -> if (alt) "Alt Encoder Position" else "Position"
    else -> TODO()
  }
}
