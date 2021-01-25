package io.excaliburfrc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final CANEncoder encoder;
  private final CANPIDController controller;
  private ShooterSpeed target = null;

  public Shooter() {
    shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
    shooterMotor.setIdleMode(IdleMode.kCoast);

    encoder = shooterMotor.getEncoder();
    encoder.setVelocityConversionFactor(ShooterConstants.GEARING);

    controller = shooterMotor.getPIDController();
    controller.setFeedbackDevice(encoder);
  }

  public void start(ShooterSpeed speed) {
    target = speed;
    controller.setReference(speed.rpm, ControlType.kVelocity);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  public boolean isAtTargetVelocity() {
    if (target == null) {
      return false;
    }
    return Math.abs(encoder.getVelocity() - target.rpm) < ShooterConstants.TOLERANCE;
  }

  public enum ShooterSpeed {
    HIGH(6000),
    LOW(2000);
    public final double rpm;

    ShooterSpeed(double i) {
      this.rpm = i;
    }
  }
}
