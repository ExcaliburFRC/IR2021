package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements AutoCloseable {
  private final WPI_VictorSPX intakeMotor;
  private final DoubleSolenoid piston;

  public Intake() {
    intakeMotor = new WPI_VictorSPX(INTAKE_MOTOR_ID);
    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD_CHANNEL, REVERSE_CHANNEL);
  }

  @Override
  public void close() throws Exception {
    intakeMotor.DestroyObject();
    piston.close();
  }

  public enum Mode {
    IN(0.6),
    AUTO(1),
    OUT(-0.4),
    OFF(0.0);

    Mode(double i) {
      speed = i;
    }

    public final double speed;
  }

  public void raise() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void lower() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public void activate(Mode speed) {
    if (isOpen()) {
      intakeMotor.set(speed.speed);
    } else {
      intakeMotor.set(Mode.OFF.speed);
    }
  }

  public boolean isOpen() {
    return piston.get() == DoubleSolenoid.Value.kForward;
  }

  public void stop() {
    intakeMotor.set(Mode.OFF.speed);
  }

  public void _DebugSet(double x) {
    intakeMotor.set(x);
  }
}
