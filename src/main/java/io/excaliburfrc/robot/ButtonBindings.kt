package io.excaliburfrc.robot

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.StartEndCommand
import io.excaliburfrc.lib.*
import io.excaliburfrc.robot.subsystems.Climber
import io.excaliburfrc.robot.subsystems.Drivetrain
import io.excaliburfrc.robot.subsystems.SuperStructure
import io.excaliburfrc.robot.subsystems.Vision
import io.excaliburfrc.robot.subsystems.Vision.CameraPosition

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
    superstructure: SuperStructure,
    climber: Climber,
    compressor: Compressor,
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

  superstructure.intake.apply {
    armJoystick(openIntakeButton).whenPressed(this) { lower() }

    armJoystick(closeIntakeButton).whenPressed(this) { raise() }
  }

  armJoystick(startShootButton)
      .toggleWhenPressed(
          superstructure.shoot(
              armJoystick.button(shootButton), armJoystick.button(ejectButton), drivetrain))

  armJoystick(startDummyShootButton)
      .toggleWhenPressed(
          superstructure.dummyShoot(
              armJoystick.button(shootButton), armJoystick.button(ejectButton)))

  val climbMode =
      climber.ClimbMode(
          { Constants.ClimberConstants.DEADBAND < armJoystick.getRawAxis(climberMotorAxis) },
          { armJoystick.getRawAxis(climberMotorAxis) < -Constants.ClimberConstants.DEADBAND })

  armJoystick(climberOpenButton).whenPressed(climber) {
    climbMode.schedule()
    climber.open()
  }
  armJoystick(climberCloseButton).whenPressed(climber) {
    climbMode.cancel()
    climber.close()
  }

  armJoystick(compressorToggle)
      .toggleWhenPressed(
          StartEndCommand(
              { compressor.closedLoopControl = false }, { compressor.closedLoopControl = true }))
  CommandScheduler.getInstance().addButton {
    SmartDashboard.putBoolean("compressor", compressor.enabled())
  }

  val vision = superstructure.vision
  armJoystick.pov(0).whenPressed(vision) { goTo(Vision.Mode.DRIVER, CameraPosition.FORWARD) }
  armJoystick.pov(180).whenPressed(vision) { goTo(Vision.Mode.TARGET, CameraPosition.UP) }
}
