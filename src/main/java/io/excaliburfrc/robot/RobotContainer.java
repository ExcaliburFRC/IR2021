package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import io.excaliburfrc.robot.commands.auto.SlalumAuto;
import io.excaliburfrc.robot.subsystems.*;

import java.util.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems, as `public final`
  public final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  public final Transporter transporter = new Transporter();
  public final Drivetrain drivetrain = new Drivetrain();
  public final Climber climber = new Climber();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick armJoystick = new Joystick(1);
  private final Compressor compressor = new Compressor();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    chooser.setDefaultOption("Nothing", new InstantCommand());
    chooser.addOption("Slalum", new SlalumAuto(drivetrain).getCommand());
    SmartDashboard.putData("Auto", chooser);
    // Configure the button bindings
    configureButtonBindings();
    initSubsystemStates();
  }

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
                () ->
                      drivetrain.arcade(
                            driveJoystick.getRawAxis(forwardDriveAxis),
                            driveJoystick.getRawAxis(rotateDriveAxis) * -1),
                drivetrain));

    new JoystickButton(armJoystick, inButton)
          .whenPressed(() -> intake.activate(Intake.Mode.IN), intake)
          .whenReleased(() -> intake.stop(), intake);
    new JoystickButton(armJoystick, openIntakeButton).whenPressed(() -> intake.lower(), intake);
    new JoystickButton(armJoystick, closeIntakeButton).whenPressed(() -> intake.raise(), intake);

    new JoystickButton(armJoystick, startShootButton)
          .toggleWhenPressed(
                new StartEndCommand(
                      () -> shooter.start(Shooter.ShooterSpeed.HIGH), () -> shooter.stop(), shooter));

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
  }

  public void initSubsystemStates() {
    intake.raise();
    intake.activate(Intake.Mode.OFF);
    drivetrain.resetPose();
  }

  public Command getAuto() {
//    Trajectory traj = TrajectoryGenerator.generateTrajectory(
//          new Pose2d(0, 0, new Rotation2d(0.0)),
//          List.of(new Translation2d(1, 0)),

//          new Pose2d(4, 0, new Rotation2d(0)), new TrajectoryConfig(3, 3)
//    );
//    return drivetrain.ramseteGroup(traj);
    return chooser.getSelected();
  }

  //  /** @deprecated - move to separate classes */
  //  @Deprecated
  //  public enum AutoPath {
  //    Slalum("slalum"),
  //    Barrel("barrel"),
  //    Bounce("bounce", "bounce1", "bounce2", "bounce3", "bounce4");
  //
  //    private static final Path outputDir =
  //        Filesystem.getDeployDirectory().toPath().resolve("output");
  //
  //    private final List<String> files;
  //    private Optional<Trajectory> trajectory = Optional.empty();
  //
  //    AutoPath(String... files) {
  //      this.files = Arrays.asList(files);
  //    }
  //
  //    public Pose2d getStartingPose() {
  //      return getTrajectory().getInitialPose();
  //    }
  //
  //    public Trajectory getTrajectory() {
  //      if (trajectory.isPresent()) return trajectory.get();
  //      if (files.size() == 1) {
  //        var res = getTrajectoryFile(files.get(0));
  //        trajectory = Optional.of(res);
  //        return res;
  //      }
  //      var res =
  //          new Trajectory(
  //              files.stream()
  //                  .flatMap(file -> AutoPath.getTrajectoryFile(file).getStates().stream())
  //                  .collect(Collectors.toList()));
  //      trajectory = Optional.of(res);
  //      return res;
  //    }
  //
  //    private static Trajectory getTrajectoryFile(String filename) {
  //      try {
  //        return TrajectoryUtil.fromPathweaverJson(outputDir.resolve(filename + ".wpilib.json"));
  //      } catch (IOException iox) {
  //        throw new RuntimeException(iox);
  //      }
  //    }
  //  }
}
