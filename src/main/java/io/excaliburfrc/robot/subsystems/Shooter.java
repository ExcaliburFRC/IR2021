package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.ShooterConstants.*;

import com.revrobotics.*;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SimpleMotorFeedforward ff;
  private ShooterSpeed target = null;

  private FlywheelSim flywheel;
  private CANEncoderSim simEncoder;

  public Shooter() {
    shooterMotor = new SimSparkMax(SHOOTER_ID, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kCoast);

    encoder = shooterMotor.getEncoder();
    encoder.setPositionConversionFactor(GEARING);
    encoder.setVelocityConversionFactor(GEARING);

    controller = shooterMotor.getPIDController();
    controller.setFeedbackDevice(encoder);
    controller.setP(kP);
    controller.setI(0.0);
    controller.setD(0.0);
    controller.setFF(0.0);
    ff = new SimpleMotorFeedforward(kS, kV, kA);

    if (RobotBase.isSimulation()) {
      flywheel =
          new FlywheelSim(
              LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
              DCMotor.getNEO(1),
              ShooterConstants.GEARING);
      simEncoder = new CANEncoderSim(false, ShooterConstants.SHOOTER_ID);
    }
    //    controller.setFeedbackDevice(encoder);

  }

  @Override
  public void simulationPeriodic() {
    var vin = shooterMotor.getAppliedOutput() * RobotController.getInputVoltage();
    SmartDashboard.putNumber("vin", vin);
    flywheel.setInputVoltage(vin);
    flywheel.update(0.02);
    simEncoder.setVelocity(flywheel.getAngularVelocityRPM());
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooterVel", encoder.getVelocity());
    SmartDashboard.putBoolean("isReady", isAtTargetVelocity());
  }
}
