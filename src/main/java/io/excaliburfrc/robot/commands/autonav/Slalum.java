package io.excaliburfrc.robot.commands.autonav;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.Drivetrain;
import java.io.IOException;

public class Slalum {
  private final Command command;
  private Trajectory traj;

  public Slalum(Drivetrain drive) {
    try {
      traj =
          TrajectoryUtil.fromPathweaverJson(
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("output")
                  .resolve("slalum.wpilib.json"));
    } catch (IOException e) {
      command = new InstantCommand();
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
      DriverStation.reportWarning("Initialized command to empty", false);
      return;
    }

    command =
        new SequentialCommandGroup(
            new InstantCommand(() -> drive.resetPose(traj.getInitialPose()), drive),
            drive.ramsete(traj),
            new InstantCommand(() -> drive.stop(), drive));
  }

  public Command getCommand() {
    return command;
  }
}
