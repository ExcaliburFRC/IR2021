package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.DriveConstants.LEFT_FOLLOWER_ID;
import static io.excaliburfrc.robot.Constants.DriveConstants.LEFT_LEADER_ID;
import static io.excaliburfrc.robot.Constants.DriveConstants.RIGHT_FOLLOWER_ID;
import static io.excaliburfrc.robot.Constants.DriveConstants.RIGHT_LEADER_ID;
import static io.excaliburfrc.robot.Constants.DriveConstants.TPS;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax rightLeader;
  private CANSparkMax rightFollower;
  private CANSparkMax leftLeader;
  private CANSparkMax leftFollower;
  private CANEncoder leftEncoder;
  private DifferentialDrive drive;

  public Drivetrain() {
    rightLeader = new CANSparkMax(RIGHT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftLeader = new CANSparkMax(LEFT_LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftFollower = new CANSparkMax(LEFT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFollower = new CANSparkMax(RIGHT_FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftEncoder = leftLeader.getAlternateEncoder(TPS);
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    drive = new DifferentialDrive(leftLeader, rightLeader);
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void arcade(double x, double r) {
    drive.arcadeDrive(x, r);
  }
}
