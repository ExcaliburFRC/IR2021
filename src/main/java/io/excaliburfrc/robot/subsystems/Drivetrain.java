package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.SimSparkMax;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax rightLeader, rightFollower;
  private final CANSparkMax leftLeader, leftFollower;
  private final CANEncoder leftEncoder;
  private final CANEncoder rightEncoder;
  private final AHRS gyro;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;

  private SimDouble simGyro;
  private DifferentialDrivetrainSim simDrive;
  private final Field2d field;

  public Drivetrain() {
    rightLeader = new SimSparkMax(RIGHT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftLeader = new SimSparkMax(LEFT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new SimSparkMax(LEFT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new SimSparkMax(RIGHT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftEncoder = leftLeader.getAlternateEncoder(CPR);
    leftEncoder.setPositionConversionFactor(PULSE_TO_METER);
    rightEncoder = rightLeader.getAlternateEncoder(CPR);
    rightEncoder.setPositionConversionFactor(PULSE_TO_METER);
    gyro = new AHRS();
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    drive = new DifferentialDrive(leftLeader, rightLeader);
    drive.setRightSideInverted(false);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    field = new Field2d();

    if (RobotBase.isSimulation()) {
      simGyro = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");
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

    leftEncoder.setPosition(simDrive.getLeftPositionMeters());
    rightEncoder.setPosition(simDrive.getRightPositionMeters());
    simGyro.set(simDrive.getHeading().getDegrees());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simDrive.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field", field);
  }

  public double getLeftEncoder() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void arcade(double x, double r) {
    drive.arcadeDrive(x, r);
  }

  public Command ramsete(Trajectory path) {
    return new RamseteCommand(
        path,
        () -> odometry.getPoseMeters(),
        new RamseteController(),
        new DifferentialDriveKinematics(TRACK_WIDTH),
        (left, right) -> {
          leftLeader.getPIDController().setReference(left, ControlType.kVelocity);
          rightLeader.getPIDController().setReference(right, ControlType.kVelocity);
        },
        this);
  }
}
