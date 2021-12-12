package io.excaliburfrc.robot.commands.galsearch;

import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.excaliburfrc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class GalacticSearch {
  static final Pose2d origin = new Pose2d();
  static final Pose2d start =
      new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), Rotation2d.fromDegrees(0));
  static final Pose2d end =
      new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(90), Rotation2d.fromDegrees(0));
  static final double[] RedA = {90, 90, 150, 60, 180, 150};
  static final double[] RedB = {90, 120, 150, 60, 210, 120};
  static final double[] BlueA = {180, 30, 210, 120, 270, 90};
  static final double[] BlueB = {180, 60, 240, 120, 300, 60};

  static List<Translation2d> convert(double[] poses) {
    var list = new ArrayList<Translation2d>(3);
    for (int i = 0; i < 3; i++) {
      Pose2d transform =
          new Pose2d(
              Units.inchesToMeters(poses[2 * i + 0]) - start.getX(),
              Units.inchesToMeters(poses[2 * i + 1]) - start.getY(),
              Rotation2d.fromDegrees(0));
      list.add(transform.getTranslation());
    }
    return list;
  }

  static Trajectory generate(List<Translation2d> innerWaypoints) {
    var config = new TrajectoryConfig(3, 3);
    return TrajectoryGenerator.generateTrajectory(start, innerWaypoints, end, config);
  }

  public static class SimulationManager implements Runnable {
    SendableChooser<Trajectory> chooser = new SendableChooser<>();
    Field2d field = new Field2d();

    public SimulationManager() {
      chooser.setDefaultOption("nothing", null);
      chooser.addOption("RedA", generate(convert(RedA)));
      chooser.addOption("RedB", generate(convert(RedB)));
      chooser.addOption("BlueA", generate(convert(BlueA)));
      chooser.addOption("BlueB", generate(convert(BlueB)));
      SmartDashboard.putData("GalacticSearch", chooser);
      SmartDashboard.delete("Field2d");
      SmartDashboard.putData("GS", field);
    }

    @Override
    public void run() {
      var sel = chooser.getSelected();
      if (sel == null) return;
      field
          .getObject("x")
          .setPoses(
              sel.getStates().stream().map(state -> state.poseMeters).collect(Collectors.toList()));
    }
  }

  private enum Path {
    AR(RedA),
    AB(BlueA),
    BR(RedB),
    BB(BlueB);

    final double[] poses;

    Path(double[] poses) {
      this.poses = poses;
    }
  }

  public static Trajectory getTrajectory(Vision vision) {
    var path = determinePath(vision.result()).poses;
    return generate(convert(path));
  }

  private static Path determinePath(PhotonPipelineResult result) {
    var distance =
        PhotonUtils.calculateDistanceToTargetMeters(
            Vision.CameraPosition.FORWARD.height, 0.1, Vision.CameraPosition.FORWARD.pitch, 0);
    if (distance < Units.inchesToMeters(100)) {
      // we're doing red!
      if (result.getTargets().get(0).getYaw() > 5) {
        return Path.BR;
      } else {
        return Path.AR;
      }
    } else {
      // we're doing blue!
      if (result.getTargets().size() == 3) {
        return Path.BB;
      } else {
        return Path.AB;
      }
    }
  }
}
