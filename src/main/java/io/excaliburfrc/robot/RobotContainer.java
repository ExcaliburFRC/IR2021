package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import io.excaliburfrc.robot.subsystems.Drivetrain;
import io.excaliburfrc.robot.subsystems.Transporter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems, as `public final`
  public final Transporter transporter = new Transporter();
  public final Drivetrain drivetrain = new Drivetrain();

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick armJoystick = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
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
}
