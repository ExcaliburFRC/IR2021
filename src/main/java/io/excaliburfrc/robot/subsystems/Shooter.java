package io.excaliburfrc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.CANEncoderSim;
import io.excaliburfrc.lib.SimSparkMax;
import io.excaliburfrc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final CANEncoder encoder;
  private final CANPIDController controller;
  private ShooterSpeed target = null;

  private CANEncoderSim simEncoder;
  private FlywheelSim flywheelSim;

  public Shooter() {
    shooterMotor = new SimSparkMax(ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
    shooterMotor.setIdleMode(IdleMode.kCoast);

    encoder = shooterMotor.getEncoder();
    encoder.setVelocityConversionFactor(ShooterConstants.GEARING);

    controller = shooterMotor.getPIDController();
    controller.setP(ShooterConstants.kP);
    controller.setI(0);
    controller.setD(0);
    controller.setFF(ShooterConstants.kF);

    if (RobotBase.isSimulation()) {
      simEncoder = new CANEncoderSim(false, ShooterConstants.SHOOTER_ID);
      flywheelSim =
          new FlywheelSim(
              LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
              DCMotor.getNEO(1),
              ShooterConstants.GEARING);
    }
  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.setInputVoltage(
        RobotController.getInputVoltage() * shooterMotor.getAppliedOutput());
    flywheelSim.update(0.02);
    simEncoder.setVelocity(flywheelSim.getAngularVelocityRPM());
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
