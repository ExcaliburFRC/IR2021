package io.excaliburfrc.robot;

import static io.excaliburfrc.robot.ButtonBindingsKt.*;
import static io.excaliburfrc.robot.subsystems.Vision.CameraPosition.UP;
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

    // armJoystick

    bind(driveJoystick, armJoystick, drivetrain, superstructure, climber, compressor);
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
