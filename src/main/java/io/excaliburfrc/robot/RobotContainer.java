package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import io.excaliburfrc.robot.commands.autonav.Slalum;
import io.excaliburfrc.robot.subsystems.*;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems, as `public final`
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Transporter transporter = new Transporter();
  public final Drivetrain drivetrain = new Drivetrain();
  public final Climber climber = new Climber();
  public final Vision vision = new Vision();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick armJoystick = new Joystick(1);
  private final Compressor compressor = new Compressor();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chooser.setDefaultOption("Nothing", new InstantCommand());
    chooser.addOption("Slalum", new Slalum(drivetrain));
    SmartDashboard.putData("Auto", chooser);
    // Configure the button bindings
    configureButtonBindings();
    initSubsystemStates();
  }

  private static final int kArcadeDrive = 0;
  private static final int kTankDrive = 1;
  private final AtomicInteger mode = new AtomicInteger(0);

  @SuppressWarnings("Convert2MethodRef")
  private void configureButtonBindings() {
    // create `JoystickButton`s binding between the buttons and commands.
    // use the two joysticks that are already declared: `driveJoystick` and `armJoystick`
    // DO NOT CREATE MORE JOYSTICKS! or rename them

    // driverJoystick
    final int forwardDriveAxis = 2;
    final int rotateDriveAxis = 1;

    // armJoystick
    final int shootButton = 1;
    final int inButton = 2;
    final int ejectButton = 4;
    final int openIntakeButton = 3;
    final int closeIntakeButton = 5;
    final int startShootButton = 6;
    final int climberOpenButton = 7;
    final int climberCloseButton = 8;
    final int climberUpButton = 9;
    final int climberDownButton = 10;
    final int compressorToggle = 12;

    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              if (mode.get() % 2 == 0)
                drivetrain.arcade(
                    driveJoystick.getRawAxis(forwardDriveAxis),
                    driveJoystick.getRawAxis(rotateDriveAxis) * -1);
              else
                drivetrain.tankDrive(
                    -driveJoystick.getRawAxis(rotateDriveAxis), driveJoystick.getRawAxis(5));
            },
            drivetrain));

    new JoystickButton(driveJoystick, 10).whenPressed(mode::incrementAndGet);

    new JoystickButton(armJoystick, inButton)
        .whenPressed(() -> intake.activate(Intake.Mode.IN), intake)
        .whenReleased(() -> intake.stop(), intake);
    new JoystickButton(armJoystick, openIntakeButton).whenPressed(() -> intake.lower(), intake);
    new JoystickButton(armJoystick, closeIntakeButton).whenPressed(() -> intake.raise(), intake);

    new JoystickButton(armJoystick, startShootButton)
        .toggleWhenPressed(
            new StartEndCommand(
                () -> shooter.start(SmartDashboard.getNumber("target_rps", 0)),
                () -> shooter.stop(),
                shooter));

    new JoystickButton(armJoystick, climberOpenButton).whenPressed(() -> climber.open(), climber);
    new JoystickButton(armJoystick, climberCloseButton).whenPressed(() -> climber.close(), climber);

    new JoystickButton(armJoystick, climberUpButton)
        .whileHeld(() -> climber.up(), climber)
        .whenReleased(() -> climber.stopMotor(), climber);

    new JoystickButton(armJoystick, climberDownButton)
        .whileHeld(() -> climber.down(), climber)
        .whenReleased(() -> climber.stopMotor(), climber);

    // TODO: merge with intake
    new JoystickButton(armJoystick, inButton)
        .whenPressed(() -> transporter.activate(Transporter.Mode.IN), transporter)
        .whenReleased(() -> transporter.activate(Transporter.Mode.OFF), transporter);

    new JoystickButton(armJoystick, ejectButton)
        .whenPressed(() -> transporter.activate(Transporter.Mode.OUT), transporter)
        .whenReleased(() -> transporter.activate(Transporter.Mode.OFF), transporter);

    // TODO: merge with shooter
    new JoystickButton(armJoystick, shootButton)
        .whenPressed(() -> transporter.activate(Transporter.Mode.SHOOT), transporter)
        .whenReleased(() -> transporter.activate(Transporter.Mode.OFF), transporter);

    new JoystickButton(armJoystick, compressorToggle)
        .toggleWhenPressed(
            new StartEndCommand(
                () -> compressor.setClosedLoopControl(false),
                () -> compressor.setClosedLoopControl(true)));

    new JoystickButton(driveJoystick, 3)
        .toggleWhenPressed(
            new InstantCommand(() -> vision.goTo(Vision.Mode.BALL, Vision.CameraPosition.FORWARD)));
  }

  public void initSubsystemStates() {
    intake.raise();
    intake.activate(Intake.Mode.OFF);
    drivetrain.resetPose();
    vision.goTo(Vision.Mode.TARGET, Vision.CameraPosition.FORWARD);
  }

  public Command getAuto() {
    return chooser.getSelected();
  }
}
