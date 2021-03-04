package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.lib.*;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax rightLeader;
  private final CANSparkMax rightFollower;
  private final CANSparkMax leftLeader;
  private final CANSparkMax leftFollower;
  private final CANEncoder leftEncoder;
  private final CANEncoder rightEncoder;
  private final AHRS gyro;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final CANPIDController leftController;
  private final CANPIDController rightController;

  private SimDouble simGyro;
  private CANEncoderSim simLeftEncoder;
  private CANEncoderSim simRightEncoder;
  private DifferentialDrivetrainSim simDrive;
  private final Field2d field;
  private final SimpleMotorFeedforward velFF = new SimpleMotorFeedforward(kS, kV_lin, kA_lin);

  public Drivetrain() {
    rightLeader = new SimSparkMax(RIGHT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftLeader = new SimSparkMax(LEFT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new SimSparkMax(LEFT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new SimSparkMax(RIGHT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    rightLeader.setInverted(true);
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftEncoder.setPositionConversionFactor(PULSE_TO_METER);
    leftEncoder.setVelocityConversionFactor(PULSE_TO_METER / 60.);
    rightEncoder.setPositionConversionFactor(PULSE_TO_METER);
    rightEncoder.setVelocityConversionFactor(PULSE_TO_METER / 60.);


    leftController = leftLeader.getPIDController();
    rightController = rightLeader.getPIDController();
    leftController.setP(kP);
    leftController.setI(0);
    leftController.setD(0);
    leftController.setFF(0);
    rightController.setP(kP);
    rightController.setI(0);
    rightController.setD(0);
    rightController.setFF(0);

    gyro = new AHRS(SPI.Port.kMXP);

    drive = new DifferentialDrive(leftLeader, rightLeader);
    drive.setRightSideInverted(true);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    field = new Field2d();

    if (RobotBase.isSimulation()) {
      simLeftEncoder = new CANEncoderSim(false, LEFT_LEADER_ID);
      simRightEncoder = new CANEncoderSim(false, RIGHT_LEADER_ID);
      simGyro =
            new SimDouble(
                  SimDeviceDataJNI.getSimValueHandle(
                        SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
      simDrive =
            new DifferentialDrivetrainSim(
                  LinearSystemId.identifyDrivetrainSystem(kV_lin, kA_lin, kV_ang, kA_ang),
                  DCMotor.getNEO(2),
                  GEARING,
                  TRACK_WIDTH,
                  WHEEL_RADIUS,
                  null);
    }
  }

  @Override
  public void simulationPeriodic() {
    var volts = RobotController.getInputVoltage();
    simDrive.setInputs(volts * leftLeader.get(), volts * rightLeader.get());

    simDrive.update(0.02);

    simLeftEncoder.setPosition(simDrive.getLeftPositionMeters());
    simLeftEncoder.setVelocity(simDrive.getLeftVelocityMetersPerSecond());
    simRightEncoder.setPosition(simDrive.getRightPositionMeters());
    simRightEncoder.setVelocity(simDrive.getRightPositionMeters());
    simGyro.set(-simDrive.getHeading().getDegrees());
    RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(simDrive.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro", gyro.getAngle());
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("RightEncoder", rightEncoder.getVelocity());
    SmartDashboard.putNumber("LeftEncoder", leftEncoder.getVelocity());
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void arcade(double x, double r) {
    drive.arcadeDrive(x, r);
  }

  public Command ramseteGroup(Trajectory path) {
    return new SequentialCommandGroup(
          new InstantCommand(() -> resetPose(path.getInitialPose()), this), // reset pose before
          ramsete(path),
          new InstantCommand(() -> this.arcade(0, 0), this)); // stop after
  }

  /**
   * Get the RamseteCommand itself, without the accompanying setup and shutdown.
   *
   * <p>Setup: <code>new InstantCommand(()->this.resetPose(path.getInitialPose()), this)</code>
   *
   * <p>Teardown: <code>new InstantCommand(()->this.arcade(0,0), this)</code>
   */
  @SuppressWarnings("Convert2MethodRef")
  public RamseteCommand ramsete(Trajectory path) {
    return new RamseteCommand(
          path,
          () -> odometry.getPoseMeters(),
          new RamseteController(),
          new SimpleMotorFeedforward(kS, kV_lin, kA_lin),
          new DifferentialDriveKinematics(TRACK_WIDTH),
          () -> {
            var whl = new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
            SmartDashboard.putNumberArray("wheelspeeds", new double[]{whl.leftMetersPerSecond, whl.rightMetersPerSecond});
            return whl;
          },
          new PIDController(kP, 0, 0),
          new PIDController(kP, 0, 0),
          (left, right) -> {
            SmartDashboard.putNumber("left", left);
            SmartDashboard.putNumber("right", right);
            leftLeader.setVoltage(left);
            rightLeader.setVoltage(right);
            drive.feed();
          },
          this);
  }

  public void resetPose(Pose2d pose) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    odometry.resetPosition(pose, gyro.getRotation2d());
    // simDrive.setPose(pose);
    field.setRobotPose(pose);
  }

  public void resetPose() {
    gyro.reset();
    resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public void setVelocityRefs(double left, double right) {
    leftController.setReference(
          left, ControlType.kVelocity, 0, velFF.calculate(left), ArbFFUnits.kVoltage);
    rightController.setReference(
          right, ControlType.kVelocity, 0, velFF.calculate(right), ArbFFUnits.kVoltage);
    SmartDashboard.putNumberArray("vel-setpoints", new double[]{left, right});
  }

  public void stop() {
    setVelocityRefs(0, 0);
    leftLeader.stopMotor();
    rightLeader.stopMotor();
  }
}
