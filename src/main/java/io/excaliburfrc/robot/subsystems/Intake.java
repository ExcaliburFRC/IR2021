package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.IntakeConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private WPI_VictorSPX intakeMotor;
  private DoubleSolenoid piston;

  public Intake() {
    intakeMotor = new WPI_VictorSPX(2);
    piston = new DoubleSolenoid(FORWARD_CHANNEL, REVERSE_CHANNEL);
  }

  public enum Mode {
    IN(0.6),
    OUT(-0.4),
    OFF(0.0);

    Mode(double i) {
      speed = i;
    }

    public final double speed;
  }

  public void raise() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public void lower() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void activate(Mode speed) {
    if (piston.get() == DoubleSolenoid.Value.kReverse) {
      intakeMotor.set(Mode.OFF.speed); //Mode.OFF.speed
    } else {
      intakeMotor.set(speed.speed);
    }
  }

  public void stop() {
    intakeMotor.set(Mode.OFF.speed);
  }
}
