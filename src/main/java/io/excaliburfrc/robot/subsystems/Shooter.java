package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.ShooterConstants.*;

import com.revrobotics.*;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.SimSparkMax;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final CANEncoder encoder;
  private final CANPIDController controller;
  private final SimpleMotorFeedforward ff;
  private ShooterSpeed target = null;

  public Shooter() {
    shooterMotor = new SimSparkMax(SHOOTER_ID, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kCoast);

    encoder = shooterMotor.getEncoder();
    encoder.setVelocityConversionFactor(GEARING);

    controller = shooterMotor.getPIDController();
    controller.setFeedbackDevice(encoder);
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  public void start(ShooterSpeed speed) {
    target = speed;
    setVelRef(speed.rpm);
  }

  private void setVelRef(double rpm) {
    controller.setReference(rpm, ControlType.kVelocity, 0, ff.calculate(rpm), ArbFFUnits.kVoltage);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  public boolean isAtTargetVelocity() {
    if (target == null) {
      return false;
    }
    return Math.abs(encoder.getVelocity() - target.rpm) < TOLERANCE;
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
