package io.excaliburfrc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.vision.Limelight;
import io.excaliburfrc.lib.vision.Limelight.DataKey;
import io.excaliburfrc.lib.vision.Limelight.SettingsKey;
import io.excaliburfrc.robot.Constants;

public class Vision extends SubsystemBase {
  private final Limelight limelight;
  private final PWM servo;

  private static final int POWER_PORT_PIPELINE = 1;
  private static final int POWER_CELL_PIPELINE = 2;

  public Vision() {
    limelight = new Limelight();
    servo = new PWM(Constants.LL_SERVO);
  }

  public void raise() {
    limelight.setData(SettingsKey.ActivePipeline, POWER_PORT_PIPELINE);
  }

  public void lower() {
    limelight.setData(SettingsKey.ActivePipeline, POWER_CELL_PIPELINE);
  }

  public double getDistance() {
    //  TODO : calibrate [regression]
    return -1;
  }

  public double getXOffset() {
    return limelight.getData(DataKey.XOffset);
  }

  public void _DebugServoControl(double position) {
    servo.setPosition(position);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LL Servo", servo.getPosition());
  }
}
