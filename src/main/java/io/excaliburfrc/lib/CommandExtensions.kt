package io.excaliburfrc.lib

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Button
import edu.wpi.first.wpilibj2.command.button.JoystickButton

operator fun GenericHID.invoke(index: Int): JoystickButton = JoystickButton(this, index)

inline fun <T : SubsystemBase> T.runCommand(crossinline command: T.() -> Unit): RunCommand =
    RunCommand({ this.command() }, this)

inline fun <T : SubsystemBase> T.instantCommand(crossinline command: T.() -> Unit): InstantCommand =
    InstantCommand({ this.command() }, this)

inline fun <T : SubsystemBase> Button.whenPressed(
    requirement: T,
    crossinline command: T.() -> Unit
): Button = whenPressed({ requirement.command() }, requirement)
