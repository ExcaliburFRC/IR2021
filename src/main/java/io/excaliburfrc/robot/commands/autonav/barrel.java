package io.excaliburfrc.robot.commands.autonav;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.Drivetrain;
import java.io.IOException;

public class barrel extends SequentialCommandGroup {
  public barrel (Drivetrain drive) {
    try {
      Trajectory traj =
            TrajectoryUtil.fromPathweaverJson(
                  Filesystem.getDeployDirectory()
                        .toPath()
                        .resolve("output")
                        .resolve("barrel.wpilib.json"));
      addCommands(
            new InstantCommand(() -> drive.resetPose(traj.getInitialPose()), drive),
            drive.ramsete(traj),
            new InstantCommand(() -> drive.stop(), drive));
    } catch (IOException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }
  }
}
