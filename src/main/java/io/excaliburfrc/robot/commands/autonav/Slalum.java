package io.excaliburfrc.robot.commands.autonav;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.Drivetrain;
import java.io.IOException;

public class Slalum extends SequentialCommandGroup {
  public Slalum(Drivetrain drive) {
    try {
      Trajectory traj =
          TrajectoryUtil.fromPathweaverJson(
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("output")
                  .resolve("slalum.wpilib.json"));
      addCommands(
          new InstantCommand(() -> drive.resetPose(traj.getInitialPose()), drive),
          drive.ramsete(traj),
          new InstantCommand(() -> drive.stop(), drive));
    } catch (IOException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }
  }
}
