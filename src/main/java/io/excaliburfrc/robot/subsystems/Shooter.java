package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.ShooterConstants.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
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

  public Shooter() {
    shooterMotor = new SimSparkMax(SHOOTER_ID, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kCoast);

    encoder = new Encoder(CHANNEL_A, CHANNEL_B);
    encoder.setDistancePerPulse(1/TICKS_TO_WHEEL_ROTATIONS);

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
    // replace with regression function
    var suggestedTarget = visionDist;

    target = suggestedTarget;
  }

  public void start(double speed) {
    if (speed < 0) throw new IllegalArgumentException("shooter target velocity can't be negative");
    target = speed;
  }

  public void start(ShooterSpeed speed) {
    target = speed.rpm;
  }

  public void stop() {
    shooterMotor.stopMotor();
    target = 0;
  }

  public void fastPeriodic() {
    double output = 0.0;

    if (target < 0)
      throw new AssertionError(
          "shooter target velocity should not be negative"); // TODO: remove for comp
    if (DriverStation.getInstance().isEnabled() && target != 0) {
      var pid = MathUtil.clamp(controller.calculate(getVelocity(), target), 0.0, 1.0);
      var feedforward = ff.calculate(target);
      output = pid + feedforward;
    }
    if (!DriverStation.getInstance().isEnabled()) target = 0;

    shooterMotor.set(output);

    // update last position for vel calculation
    previousPosition = encoder.getDistance();

    SmartDashboard.putNumber("shooterPos", encoder.getDistance());
    SmartDashboard.putNumber("shooterVel", getVelocity());
    SmartDashboard.putBoolean("isReady", isAtTargetVelocity());
  }

  /** RPS: rounds per seconds */
  public double getVelocity() {
    // difference in position over time
    // dx/dt
    return (encoder.getDistance() - previousPosition) / kTimestep;
  }

  public boolean isAtTargetVelocity() {
    return Math.abs(getVelocity() - target) < TOLERANCE;
  }

  public void _DebugSetVel(double v) {
    shooterMotor.set(v);
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
  }
}
