package io.excaliburfrc.robot

import edu.wpi.first.wpilibj.GenericHID
import io.excaliburfrc.lib.*
import io.excaliburfrc.robot.subsystems.Drivetrain
import io.excaliburfrc.robot.subsystems.SuperStructure

// driverJoystick
const val forwardDriveAxis = 1

const val rotateDriveAxis = 2

const val inButton = 2

const val ejectButton = 4
// armJoystick
const val shootButton = 1

const val openIntakeButton = 3

const val closeIntakeButton = 5

const val startShootButton = 6

const val startDummyShootButton = 9

const val climberOpenButton = 8

const val climberCloseButton = 7

const val climberMotorAxis = 1

const val compressorToggle = 12

fun bind(
    driveJoystick: GenericHID,
    armJoystick: GenericHID,
    drivetrain: Drivetrain,
    superstructure: SuperStructure
) {
  drivetrain.defaultCommand =
      drivetrain.runCommand {
        arcade(
            -driveJoystick.getRawAxis(forwardDriveAxis), driveJoystick.getRawAxis(rotateDriveAxis))
      }

  armJoystick(inButton)
      .whileHeld(superstructure::intake, superstructure)
      .whenReleased(superstructure::stop, superstructure)

  armJoystick(ejectButton)
      .whileHeld(superstructure::eject, superstructure)
      .whenReleased(superstructure::stop, superstructure)

  superstructure.intake.apply { armJoystick(openIntakeButton).whenPressed(this) { lower() } }
}
