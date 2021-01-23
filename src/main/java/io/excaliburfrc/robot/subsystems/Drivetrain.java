package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax rightLeader;
  private CANSparkMax rightFollower;
  private CANSparkMax leftLeader;
  private CANSparkMax leftFollower;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private AHRS gyro;

  private DifferentialDrive drive;
  private DifferentialDriveOdometry odometry;

  public Drivetrain() {
    rightLeader = new CANSparkMax(RIGHT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftLeader = new CANSparkMax(LEFT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new CANSparkMax(LEFT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new CANSparkMax(RIGHT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftEncoder = leftLeader.getAlternateEncoder(TPS);
    rightEncoder = rightLeader.getAlternateEncoder(TPS);
    gyro = new AHRS();
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    drive = new DifferentialDrive(leftLeader, rightLeader);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
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
        () -> {
          return odometry.getPoseMeters();
        },
        new RamseteController(),
        new DifferentialDriveKinematics(TRACK_WIDTH),
        (left, right) -> {
          leftLeader.getPIDController().setReference(left, ControlType.kVelocity);
          rightLeader.getPIDController().setReference(right, ControlType.kVelocity);
        },
        this);
  }
}
