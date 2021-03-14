package io.excaliburfrc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.*;

public class ShootProcess {
  public static final Command f(
      Drivetrain drive, Shooter shoot, Transporter transporter, Vision vision) {
    PIDController x = null;
    var dtpid = new PIDCommand(x, vision::getDistance, 0, pow -> drive.arcade(0, pow));
    var shootpid =
        new FunctionalCommand(
            null, () -> shoot.startToVisionDistance(vision.getDistance()), null, null);

    return null;
  }
}
