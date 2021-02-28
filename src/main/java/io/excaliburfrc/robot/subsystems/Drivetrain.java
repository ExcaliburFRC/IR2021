package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.RamseteController;
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
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftEncoder.setPositionConversionFactor(PULSE_TO_METER);
    leftEncoder.setVelocityConversionFactor(PULSE_TO_METER);
    rightEncoder.setPositionConversionFactor(PULSE_TO_METER);
    rightEncoder.setVelocityConversionFactor(PULSE_TO_METER);

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
    drive.setRightSideInverted(false);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    field = new Field2d();

    if (RobotBase.isSimulation()) {
      simLeftEncoder = new CANEncoderSim(false, LEFT_LEADER_ID);
      // simRightEncoder = new EncoderSim(rightEncoder);
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

    var leftPositionMeters = simDrive.getLeftPositionMeters();
    // simLeftEncoder.setDistance(leftPositionMeters);
    // System.out.println(leftEncoder.getDistance() == leftPositionMeters);
    // var rightPositionMeters = simDrive.getRightPositionMeters();
    // simRightEncoder.setDistance(rightPositionMeters);
    // System.out.println(rightEncoder.getDistance() == rightPositionMeters);
    // simLeftEncoder.setRate(simDrive.getLeftVelocityMetersPerSecond());
    // simRightEncoder.setRate(simDrive.getRightVelocityMetersPerSecond());
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
    SmartDashboard.putNumber("RightEncoder", rightEncoder.getPosition());
    SmartDashboard.putNumber("LeftEncoder", leftEncoder.getPosition());
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
  public RamseteCommand ramsete(Trajectory path) {
    return new RamseteCommand(
        path,
        () -> odometry.getPoseMeters(),
        new RamseteController(),
        // new SimpleMotorFeedforward(kS, kV_lin),
        new DifferentialDriveKinematics(TRACK_WIDTH),
        // () -> new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate()),
        // new PIDController(kP, kI, kD),
        // new PIDController(kP, kI, kD),
        (left, right) -> setVelocityRefs(left, right),
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
  }
}
