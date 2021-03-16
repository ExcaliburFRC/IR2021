@file:JvmName("Main")

package io.excaliburfrc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

fun main() {
  RobotBase.startRobot { Robot }
}

const val leftX = 0

const val leftY = 1

const val rightX = 2

const val rightY = 5

/**
 * This is a sample program to demonstrate the use of state-space classes in robot simulation. This
 * robot has a flywheel, elevator, arm and differential drivetrain, and interfaces with the sim
 * GUI's [edu.wpi.first.wpilibj.smartdashboard.Field2d] class.
 */
object Robot : TimedRobot() {
  private val joystick = Joystick(0)
  private val chooser: SendableChooser<Command> by lazy {
    with(SendableChooser<Command>()) {
      setDefaultOption(
          "Arcade Drive",
          Drivetrain.arcadeDrive({ -joystick.getRawAxis(leftY) }, { joystick.getRawAxis(rightX) }))
      addOption(
          "Tank Drive",
          Drivetrain.tankDrive({ -joystick.getRawAxis(leftY) }, { -joystick.getRawAxis(rightY) }))
      addOption(
          "Left Arcade Drive",
          Drivetrain.arcadeDrive({ -joystick.getRawAxis(rightY) }, { joystick.getRawAxis(leftX) }))

      // anything else?
      // - trigger drive?
      // - curvature/cheesy drive?

      return@with this
    }
  }

  override fun robotInit() {
    Drivetrain
    SmartDashboard.putData("Drive Type", chooser)
  }

  override fun teleopInit() {
    chooser.selected?.schedule()
        ?: DriverStation.reportError("Selected was null for some flipping reason", true)
  }

  override fun teleopPeriodic() {
    CommandScheduler.getInstance().run()
  }
}
