package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;
import java.util.function.BooleanSupplier;

public class Climber extends SubsystemBase {
  private final WPI_TalonSRX leader;
  private final WPI_TalonSRX follower;
  private final DoubleSolenoid hanger;

  public Climber() {
    leader = new WPI_TalonSRX(LEADER_ID);
    follower = new WPI_TalonSRX(FOLLOWER_ID);
    hanger = new DoubleSolenoid(HANGER_FWD, HANGER_REV);
    leader.setInverted(true);
    follower.follow(leader);
    follower.setInverted(InvertType.OpposeMaster);
  }

  public Command ClimbMode(BooleanSupplier up, BooleanSupplier down) {
    return new FunctionalCommand(
        () -> {}, // init
        () -> { // exe
          LEDs.INSTANCE.setMode(LedMode.RAINBOW);
          if (up.getAsBoolean()) up();
          else if (down.getAsBoolean()) down();
          else stopMotor();
        },
        interrupted -> stopMotor(), // end
        () -> false, // isFinished
        this);
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
