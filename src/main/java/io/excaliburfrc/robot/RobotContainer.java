package io.excaliburfrc.robot;

import static io.excaliburfrc.robot.subsystems.Vision.CameraPosition.FORWARD;
import static io.excaliburfrc.robot.subsystems.Vision.CameraPosition.UP;
import static io.excaliburfrc.robot.subsystems.Vision.Mode.DRIVER;
import static io.excaliburfrc.robot.subsystems.Vision.Mode.TARGET;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  public final Drivetrain drivetrain = new Drivetrain();
  public final SuperStructure superstructure = new SuperStructure();

  public final Climber climber = new Climber();

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

  private final AtomicInteger mode = new AtomicInteger(0);

  @SuppressWarnings("Convert2MethodRef")
  private void configureButtonBindings() {
    // create `JoystickButton`s binding between the buttons and commands.
    // use the two joysticks that are already declared: `driveJoystick` and `armJoystick`3
    // DO NOT CREATE MORE JOYSTICKS! or rename them

    // driverJoystick
    final int forwardDriveAxis = 1;
    final int rotateDriveAxis = 2;

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
            () ->
                drivetrain.arcade(
                    -driveJoystick.getRawAxis(forwardDriveAxis),
                    driveJoystick.getRawAxis(rotateDriveAxis)),
            drivetrain));

    new JoystickButton(driveJoystick, 10).whenPressed(mode::incrementAndGet);

    new JoystickButton(armJoystick, inButton)
        .whenPressed(superstructure::intake, superstructure)
        .whenReleased(superstructure::stop, superstructure);
    new JoystickButton(armJoystick, ejectButton)
        .whenPressed(() -> superstructure.eject(), superstructure)
        .whenReleased(() -> superstructure.stop(), superstructure);

    var intake = superstructure.intake;
    new JoystickButton(armJoystick, openIntakeButton).whenPressed(() -> intake.lower(), intake);
    new JoystickButton(armJoystick, closeIntakeButton).whenPressed(() -> intake.raise(), intake);

    new JoystickButton(armJoystick, startShootButton)
        .toggleWhenPressed(
            superstructure.shoot(() -> armJoystick.getRawButton(shootButton), drivetrain));

    new JoystickButton(armJoystick, climberOpenButton).whenPressed(() -> climber.open(), climber);
    new JoystickButton(armJoystick, climberCloseButton).whenPressed(() -> climber.close(), climber);

    new JoystickButton(armJoystick, climberUpButton)
        .whileHeld(() -> climber.up(), climber)
        .whenReleased(() -> climber.stopMotor(), climber);

    new JoystickButton(armJoystick, climberDownButton)
        .whileHeld(() -> climber.down(), climber)
        .whenReleased(() -> climber.stopMotor(), climber);

    new JoystickButton(armJoystick, compressorToggle)
        .toggleWhenPressed(
            new StartEndCommand(
                () -> compressor.setClosedLoopControl(false),
                () -> compressor.setClosedLoopControl(true)));
    CommandScheduler.getInstance()
        .addButton(() -> SmartDashboard.putBoolean("compressor", compressor.enabled()));

    var vision = superstructure.vision;
    new POVButton(armJoystick, 0).whenPressed(() -> vision.goTo(TARGET, FORWARD), vision);
    new POVButton(armJoystick, 270).whenPressed(() -> vision.goTo(DRIVER, FORWARD), vision);
    new POVButton(armJoystick, 180).whenPressed(() -> vision.goTo(TARGET, UP), vision);
  }

  public void initSubsystemStates() {
    superstructure.init();
    drivetrain.resetPose();
  }

  public Command getAuto() {
    return chooser.getSelected();
  }
}
