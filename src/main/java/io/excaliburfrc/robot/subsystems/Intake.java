package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.IntakeConstants.*;

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
  public void raisePisto(){
    piston.set(DoubleSolenoid.Value.kForward);
  }
  public void lowerPisto(){
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void openPiston() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public void closePiston() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public void activate(Mode speed) {
    if (piston.get() == DoubleSolenoid.Value.kReverse) {
      intakeMotor.set(Mode.OFF.speed);
    } else {
      intakeMotor.set(speed.speed);
    }
  }

  public void stop() {
    intakeMotor.set(Mode.OFF.speed);
  }
}
