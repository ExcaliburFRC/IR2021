package io.excaliburfrc.robot;

import static io.excaliburfrc.robot.subsystems.Vision.CameraPosition.UP;
import static io.excaliburfrc.robot.subsystems.Vision.Mode.TARGET;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.excaliburfrc.robot.subsystems.Transporter;

/**
 * This is a sample program to demonstrate the use of state-space classes in robot simulation. This
 * robot has a flywheel, elevator, arm and differential drivetrain, and interfaces with the sim
 * GUI's {@link edu.wpi.first.wpilibj.smartdashboard.Field2d} class.
 */
public class Robot extends TimedRobot {
  public RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();
    // Instantiate our RobotContainer. This will perform all our button bindings
    m_robotContainer = new RobotContainer();
    addPeriodic(
        () -> m_robotContainer.superstructure.shooter.fastPeriodic(),
        m_robotContainer.superstructure.shooter.kTimestep,
        0.003);
  }

  @Override
  public void simulationPeriodic() {
    // periodic robot-wide sim
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.getAuto().schedule();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.initSubsystemStates();
    // init teleop
  }

  @Override
  public void testInit() {
    SmartDashboard.putNumber("target_rps", 0);
    var vision = m_robotContainer.superstructure.vision;
    vision.goTo(TARGET, UP);
  }

  @Override
  public void testPeriodic() {

    final Transporter transporter = m_robotContainer.superstructure.transporter;
    if (DriverStation.getInstance().getStickButton(1, 1)) {
      transporter.activate(Transporter.Mode.SHOOT);
    } else transporter.stop();
    m_robotContainer.superstructure.shooter.start(SmartDashboard.getNumber("target_rps", 0));
  }
}
