package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.ShooterConstants.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import io.excaliburfrc.lib.SimSparkMax;
import io.excaliburfrc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final Encoder encoder;
  private final PIDController controller;
  private final SimpleMotorFeedforward ff;
  public final double kTimestep = 0.005;

  private double target = 0.0;
  private double previousPosition = 0.0;

  private FlywheelSim flywheel;
  private EncoderSim simEncoder;
  private double velocity = 0;

  public Shooter() {
    shooterMotor = new SimSparkMax(SHOOTER_ID, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.enableVoltageCompensation(12);

    encoder = new Encoder(CHANNEL_A, CHANNEL_B);
    encoder.setDistancePerPulse(1 / TICKS_TO_WHEEL_ROTATIONS);

    controller = new PIDController(kP, 0.0, 0.0, kTimestep);
    ff = new SimpleMotorFeedforward(kS, kV, kA);

    if (RobotBase.isSimulation()) {
      flywheel =
          new FlywheelSim(
              LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
              DCMotor.getNEO(1),
              ShooterConstants.TICKS_TO_WHEEL_ROTATIONS);
      simEncoder = new EncoderSim(encoder);
    }
  }

  @Override
  public void simulationPeriodic() {
    var vin = shooterMotor.getAppliedOutput() * RobotController.getInputVoltage();
    SmartDashboard.putNumber("vin", vin);
    flywheel.setInputVoltage(vin);
    flywheel.update(0.02);
    simEncoder.setRate(flywheel.getAngularVelocityRPM());
  }

  public void startToVisionDistance(double visionDist) {
    if (visionDist == -1) {
      DriverStation.reportWarning("Trying to accelerate shooter to invalid vision distance", false);
      return;
    }
    // regression function
    var suggestedTarget = 52.7 + 0.319 * visionDist + 0.614 * visionDist * visionDist;
    target = suggestedTarget;
  }

  public void start(double speed) {
    if (speed < 0) {
      DriverStation.reportWarning("Trying to accelerate shooter to invalid vision distance", false);
      return;
    }

    target = speed;
  }

  public void stop() {
    shooterMotor.stopMotor();
    target = 0;
  }

  public void fastPeriodic() {
    double output = 0.0;
    velocity = getVelocity();

    if (target < 0) { // this never happened in comp - it's safe!
      throw new AssertionError("shooter target velocity should not be negative");
    }
    if (!DriverStation.getInstance().isEnabled()) target = 0;
    if (Double.compare(target, 0.0) != 0) {
      var pid = MathUtil.clamp(controller.calculate(velocity, target), 0.0, 1.0);
      var feedforward = kF * (target);
      output = pid + feedforward;
    }

    shooterMotor.set(output);

    // update last position for vel calculation
    previousPosition = encoder.getDistance();

    SmartDashboard.putNumber("shooterVel", velocity);
    SmartDashboard.putNumber("target", target);
    SmartDashboard.putBoolean("isAtTargetVelocity", isAtTargetVelocity());
    SmartDashboard.putBoolean("isShooterActive", Double.compare(target, 0.0) != 0);
  }

  /** RPS: rounds per seconds */
  public double getVelocity() {
    // difference in position over time
    // dx/dt
    return (encoder.getDistance() - previousPosition) / kTimestep;
  }

  public boolean isAtTargetVelocity() {
    if (Double.compare(target, 0) == 0) return false;
    return Math.abs(velocity - target) < TOLERANCE;
  }
}
