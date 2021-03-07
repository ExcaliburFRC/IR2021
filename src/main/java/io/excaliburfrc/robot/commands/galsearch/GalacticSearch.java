package io.excaliburfrc.robot.commands.galsearch;

import edu.wpi.first.wpilibj.util.Units;

public class GalacticSearch {
  private enum Path {
    AR,
    AB,
    BR,
    BB;
  }

  private Path determinePath(double distance, double xOffset) {
    if (distance < Units.inchesToMeters(100)) {
      // we're doing red!
      if (xOffset > 20) {
        return Path.BR;
      } else {
        return Path.AR;
      }
    } else {
      // we're doing blue!
      if (xOffset > 16) {
        return Path.AB;
      } else {
        return Path.BB;
      }
    }
  }
}
