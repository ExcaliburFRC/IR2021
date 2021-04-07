package io.excaliburfrc.robot.commands.autonav;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.Drivetrain;
import java.io.IOException;
import java.util.ArrayList;

public class Bounce extends SequentialCommandGroup {
  public Bounce(Drivetrain drive) {
    var list = new ArrayList<RamseteCommand>();
    Pose2d startPose = new Pose2d();
    String[] names = {
      "bounce1.wpilib.json", "bounce2.wpilib.json", "bounce3.wpilib.json", "bounce4.wpilib.json"
    };
    for (int i = 0; i < names.length; i++) {
      String s = names[i];
      Trajectory t = build(s);
      if (i == 0) startPose = t.getInitialPose();
      RamseteCommand ramsete = drive.ramsete(t);
      list.add(ramsete);
    }
    Pose2d finalStartPose = startPose;
    addCommands(new InstantCommand(() -> drive.resetPose(finalStartPose), drive));
    list.forEach(this::addCommands);
    addCommands(new InstantCommand(() -> drive.stop(), drive));
  }

  private Trajectory build(String name) {
    try {
      return TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("output").resolve(name));

    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
