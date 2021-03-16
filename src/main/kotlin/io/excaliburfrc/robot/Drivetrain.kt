package io.excaliburfrc.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PWM
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import io.excaliburfrc.robot.Drivetrain.LEDs.LedMode.*
import io.excaliburfrc.robot.constants.*
import kotlin.math.abs
import kotlin.properties.Delegates

object Drivetrain : SubsystemBase() {
  private val leftLeader = CANSparkMax(LEFT_LEADER_ID, kBrushless)
  private val leftFollower = CANSparkMax(LEFT_FOLLOWER_ID, kBrushless)
  private val rightLeader = CANSparkMax(RIGHT_LEADER_ID, kBrushless)
  private val rightFollower = CANSparkMax(RIGHT_FOLLOWER_ID, kBrushless)
  init {
    val motors = setOf(leftLeader, leftFollower, rightLeader, rightFollower)
    motors.forEach { it.restoreFactoryDefaults() }
    rightLeader.inverted = true
    leftFollower.follow(leftLeader)
    rightFollower.follow(rightLeader)
    motors.forEach { it.idleMode = CANSparkMax.IdleMode.kCoast }
  }

  private val drive = DifferentialDrive(leftLeader, rightLeader)
  init {
    drive.isRightSideInverted = false
  }

  fun arcadeDrive(xSpeed: () -> Double, zRotate: () -> Double) =
      RunCommand({ drive.arcadeDrive(xSpeed(), zRotate()) }, this)
  fun tankDrive(left: () -> Double, right: () -> Double) =
      RunCommand({ drive.tankDrive(left(), right()) }, this)

  override fun periodic() {
    var v = leftLeader.get() + rightLeader.get()
    v /= 2.0
    LEDs.mode =
        when {
          DriverStation.getInstance().isDisabled -> YELLOW
          defaultCommand?.isScheduled ?: false -> OFF
          v espilonEquals 0.0 -> BLUE
          v < 0.0 -> RED
          v > 0.0 -> GREEN
          else -> RAINBOW
        }
  }

  private object LEDs {
    private val leds = PWM(LED_PORT)

    enum class LedMode(val value: Double) {
      BLUE(0.87),
      RED(0.61),
      GREEN(0.73),
      YELLOW(0.67),
      RAINBOW(-0.97),
      OFF(0.99)
    }

    var mode: LedMode by Delegates.observable(BLUE) { _, _, newValue ->
      leds.speed = newValue.value
    }
  }
}

private infix fun Double.espilonEquals(i: Double) = abs(this - i) < 1e-2
