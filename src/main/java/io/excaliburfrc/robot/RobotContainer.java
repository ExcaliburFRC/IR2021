package io.excaliburfrc.robot;

import static io.excaliburfrc.robot.subsystems.Vision.CameraPosition.FORWARD;
import static io.excaliburfrc.robot.subsystems.Vision.CameraPosition.UP;
import static io.excaliburfrc.robot.subsystems.Vision.Mode.DRIVER;
import static io.excaliburfrc.robot.subsystems.Vision.Mode.TARGET;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.Constants.ClimberConstants;
import io.excaliburfrc.robot.commands.autonav.Barrel;
import io.excaliburfrc.robot.commands.autonav.Bounce;
import io.excaliburfrc.robot.commands.autonav.Slalum;
import io.excaliburfrc.robot.subsystems.*;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;
import java.util.List;

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
    chooser.setDefaultOption("Nothing", new InstantCommand()); // for skills
    chooser.addOption("Slalum", new Slalum(drivetrain));
    chooser.addOption("Bounce", new Bounce(drivetrain));
    chooser.addOption("Barrel", new Barrel(drivetrain));
    // for competition
    // go 1 meter forward, and then shoot
    var competition =
        (new SelectCommand(
                    () ->
                        drivetrain.ramseteGroup(
                            TrajectoryGenerator.generateTrajectory(
                                List.of(
                                    new Pose2d(0, 0, new Rotation2d(0)),
                                    new Pose2d(0.6, 0, Rotation2d.fromDegrees(0))),
                                new TrajectoryConfig(3, 3))))
                .alongWith(
                    new InstantCommand(
                        () -> superstructure.vision.goTo(TARGET, UP), superstructure.vision)))
            .andThen(superstructure.shoot(() -> true, () -> false, drivetrain));
    chooser.addOption("Competition", competition);
    chooser.addOption("GalacticSearch", galacticSearch());
    SmartDashboard.putData("Auto", chooser); // for skills only
    // Configure the button bindings
    configureButtonBindings();
    initSubsystemStates();
  }

  public Command galacticSearch() {
    var ramsete = new InstantCommand(); // new
    // SelectCommand(()->drivetrain.ramseteGroup(GalacticSearch.getTrajectory(superstructure.vision)));
    var in = new RunCommand(superstructure::intake, superstructure);
    return new ParallelDeadlineGroup(ramsete, in);
  }

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
    final int startDummyShootButton = 9;
    final int climberOpenButton = 8;
    final int climberCloseButton = 7;
    final int climberMotorAxis = 1;
    final int compressorToggle = 12;

    drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                drivetrain.arcade(
                    -driveJoystick.getRawAxis(forwardDriveAxis),
                    driveJoystick.getRawAxis(rotateDriveAxis)),
            drivetrain));

    new JoystickButton(armJoystick, inButton)
        .whileHeld(superstructure::intake, superstructure)
        .whenReleased(superstructure::stop, superstructure);
    new JoystickButton(armJoystick, ejectButton)
        .whileHeld(() -> superstructure.eject(), superstructure)
        .whenReleased(() -> superstructure.stop(), superstructure);

    var intake = superstructure.intake;
    new JoystickButton(armJoystick, openIntakeButton).whenPressed(() -> intake.lower(), intake);
    new JoystickButton(armJoystick, closeIntakeButton).whenPressed(() -> intake.raise(), intake);

    new JoystickButton(armJoystick, startShootButton)
        .toggleWhenPressed(
            superstructure.shoot(
                () -> armJoystick.getRawButton(shootButton),
                () -> armJoystick.getRawButton(ejectButton),
                drivetrain));
    new JoystickButton(armJoystick, startDummyShootButton)
        .toggleWhenPressed(
            superstructure.dummyShoot(
                () -> armJoystick.getRawButton(shootButton),
                () -> armJoystick.getRawButton(ejectButton)));

    Command climbMode =
        climber.ClimbMode(
            () -> armJoystick.getRawAxis(climberMotorAxis) > ClimberConstants.DEADBAND,
            () -> armJoystick.getRawAxis(climberMotorAxis) < -ClimberConstants.DEADBAND);
    new JoystickButton(armJoystick, climberOpenButton)
        .whenPressed(
            () -> {
              climber.open();
              climbMode.schedule();
            },
            climber);
    new JoystickButton(armJoystick, climberCloseButton)
        .whenPressed(
            () -> {
              climber.close();
              climbMode.cancel();
            },
            climber);

    new JoystickButton(armJoystick, compressorToggle)
        .toggleWhenPressed(
            new StartEndCommand(
                () -> compressor.setClosedLoopControl(false),
                () -> compressor.setClosedLoopControl(true)));
    CommandScheduler.getInstance()
        .addButton(() -> SmartDashboard.putBoolean("compressor", compressor.enabled()));

    var vision = superstructure.vision;
    new POVButton(armJoystick, 0).whenPressed(() -> vision.goTo(DRIVER, FORWARD), vision);
    new POVButton(armJoystick, 180).whenPressed(() -> vision.goTo(TARGET, UP), vision);
  }

  public void initSubsystemStates() {
    superstructure.init();
    drivetrain.resetPose();
    LEDs.INSTANCE.setMode(LedMode.BLUE);
    climber.close();
    drivetrain.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public Command getAuto() {
    return chooser.getSelected(); // for skills
  }
}
