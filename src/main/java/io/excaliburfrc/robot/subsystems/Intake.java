package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.intakeConstans.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private DoubleSolenoid piston;

  public Intake() {
    intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    piston = new DoubleSolenoid(FORWARD_CHANNEL, REVERSE_CHANNEL);
  }

  public enum Mode {
    IN(0.6),
    OUT(-0.4),
    OFF(0);

    Mode(double i) {
      speed = i;
    }

    public final double speed;
  }

  public void intake(Mode speed) {
    if (piston.get() == DoubleSolenoid.Value.kReverse) {
      intakeMotor.set(speed.speed);
    } else {
      intakeMotor.set(Mode.OFF.speed);
    }
  }
}
