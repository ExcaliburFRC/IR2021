package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private WPI_TalonSRX leader;
  private WPI_TalonSRX follower;
  private DoubleSolenoid hanger;

  public Climber() {
    leader = new WPI_TalonSRX(LEADER_ID);
    follower = new WPI_TalonSRX(FOLLOWER_ID);
    hanger = new DoubleSolenoid(HANGER_FWD, HANGER_REV);
    follower.follow(leader);
  }

  public void open() {
    hanger.set(DoubleSolenoid.Value.kForward);
  }

  public void close() {
    hanger.set(DoubleSolenoid.Value.kReverse);
  }

  public void up() {
    leader.set(UP_SPEED);
  }

  public void down() {
    leader.set(DOWN_SPEED);
  }

  public void stopMotor() {
    leader.set(0);
  }
}
