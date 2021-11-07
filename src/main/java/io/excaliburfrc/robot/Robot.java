package io.excaliburfrc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.excaliburfrc.robot.commands.galsearch.GalacticSearch;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;

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
    // register the 5ms shooter velocity PID loop
    addPeriodic(
        () -> m_robotContainer.superstructure.shooter.fastPeriodic(),
        m_robotContainer.superstructure.shooter.kTimestep,
        0.003);
  }

  @Override
  public void simulationInit() {
    new GalacticSearch.SimulationManager();
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
    m_robotContainer.initSubsystemStates();
    LEDs.INSTANCE.setMode(LedMode.YELLOW);
    m_robotContainer.drivetrain.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_robotContainer.getAuto().schedule();
  }

  double stopTime = 0;

  @Override
  public void disabledInit() {
    stopTime = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.isLimited.set(true);
  }

  @Override
  public void disabledPeriodic() {
    if (Timer.getFPGATimestamp() - stopTime > 1.0) {
      m_robotContainer.drivetrain.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.initSubsystemStates();
    // init teleop
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  //    final Transporter transporter = m_robotContainer.superstructure.transporter;
  //    if (DriverStation.getInstance().getStickButton(1, 1)) {
  //      transporter.activate(Transporter.Mode.SHOOT);
  //    } else transporter.stop();
  //    m_robotContainer.superstructure.shooter.start(SmartDashboard.getNumber("target_rps", 0));
  //  }
}
