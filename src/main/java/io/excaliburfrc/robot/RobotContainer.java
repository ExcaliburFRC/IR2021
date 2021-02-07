package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import io.excaliburfrc.robot.subsystems.Drivetrain;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.Intake.Mode;
import io.excaliburfrc.robot.subsystems.Transporter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems, as `public final`
  public final Intake intake = new Intake();
  public final Transporter transporter = new Transporter();
  public final Drivetrain drivetrain = new Drivetrain();

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick armJoystick = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    initSubsystemStates();
  }

  private void configureButtonBindings() {
    new JoystickButton(armJoystick, 3)
        .whenPressed(
            () -> {
              intake.activate(Intake.Mode.IN);
            },
            intake)
        .whenReleased(
            () -> {
              intake.stop();
            },
            intake);

    new JoystickButton(armJoystick, 4).whenActive(() -> intake.raise(), intake);
    new JoystickButton(armJoystick, 5).whenPressed(() -> intake.lower(), intake);

    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              drivetrain.arcade(driveJoystick.getRawAxis(1), driveJoystick.getRawAxis(2));
            },
            drivetrain));

    // create `JoystickButton`s binding between the buttons and commands.
    // use the two joysticks that are already declared: `driveJoystick` and `armJoystick`
    // DO NOT CREATE MORE JOYSTICKS! or rename them
    final JoystickButton in = new JoystickButton(armJoystick, 1);
    final JoystickButton out = new JoystickButton(armJoystick, 2);

    in.whenPressed(
        () -> {
          transporter.setFlicker(Transporter.Mode.IN);
          transporter.setLoading(Transporter.Mode.IN);
        },
        transporter);
    in.whenReleased(
        () -> {
          transporter.setFlicker(Transporter.Mode.OFF);
          transporter.setLoading(Transporter.Mode.OFF);
        },
        transporter);
    out.whenPressed(
        () -> {
          transporter.setFlicker(Transporter.Mode.OUT);
          transporter.setLoading(Transporter.Mode.OUT);
        },
        transporter);
    out.whenReleased(
        () -> {
          transporter.setFlicker(Transporter.Mode.OFF);
          transporter.setLoading(Transporter.Mode.OFF);
        },
        transporter);
  }

  private void initSubsystemStates() {
    intake.raise();
    intake.activate(Mode.OFF);
  }
}
