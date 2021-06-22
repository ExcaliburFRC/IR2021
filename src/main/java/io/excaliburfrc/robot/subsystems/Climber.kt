package io.excaliburfrc.robot.subsystems.kt

import com.ctre.phoenix.motorcontrol.InvertType
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import io.excaliburfrc.robot.Constants.ClimberConstants
import io.excaliburfrc.robot.subsystems.LEDs
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX as Talon

object Climber : SubsystemBase() {
  private val leader = Talon(ClimberConstants.LEADER_ID)
  private val follower = Talon(ClimberConstants.FOLLOWER_ID)
  private val hanger = DoubleSolenoid(ClimberConstants.HANGER_FWD, ClimberConstants.HANGER_REV)

  init {
    leader.inverted = true
    follower.follow(leader)
    follower.setInverted(InvertType.OpposeMaster)
  }

  fun ClimbMode(upButton: () -> Boolean, downButton: () -> Boolean): Command = FunctionalCommand(
    {},  // init
    {
      // exe
      LEDs.mode = LEDs.LedMode.RAINBOW
      if (upButton()) up() else if (downButton()) down() else stopMotor()
    },
    { stopMotor() },  // end
    { false },  // isFinished
    this
  )

  fun open() = hanger.set(DoubleSolenoid.Value.kForward)

  fun close() = hanger.set(DoubleSolenoid.Value.kReverse)

  fun up() = leader.set(ClimberConstants.UP_SPEED)

  fun down() = leader.set(ClimberConstants.DOWN_SPEED)

  fun stopMotor() = leader.set(0.0)
}
