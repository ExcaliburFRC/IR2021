package io.excaliburfrc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;

public class SuperStructure extends SubsystemBase {
  public final Intake intake = new Intake();
  public final Transporter transporter = new Transporter();
  public final Shooter shooter = new Shooter();
  public final Vision vision = new Vision();

  public void init() {
    intake.raise();
    intake.activate(Intake.Mode.OFF);
    vision.goTo(Vision.Mode.DRIVER, Vision.CameraPosition.FORWARD);
  }

  public void intake() {
    intake.activate(Intake.Mode.IN);
    transporter.activate(Transporter.Mode.IN);
  }

  public void stop() {
    intake.stop();
    transporter.stop();
  }

  public void eject() {
    transporter.activate(Transporter.Mode.OUT);
  }

  public class ShootCommand extends CommandBase {
    private final BooleanSupplier trigger;
    private final BooleanSupplier isDriveLocked;

    public ShootCommand(BooleanSupplier trigger, BooleanSupplier isDriveLocked) {
      addRequirements(shooter, transporter);
      this.trigger = trigger;
      this.isDriveLocked = isDriveLocked;
    }

    @Override
    public void initialize() {
      shooter.startToVisionDistance(vision.getDistance());
    }

    @Override
    public void execute() {
      final boolean isDriveLockedAsBoolean = isDriveLocked.getAsBoolean();
      SmartDashboard.putBoolean("isDriveAligned", isDriveLockedAsBoolean);
      if (trigger.getAsBoolean() && isDriveLockedAsBoolean && shooter.isAtTargetVelocity()) {
        transporter.activate(Transporter.Mode.SHOOT);
      } else transporter.stop();
    }

    @Override
    public void end(boolean interrupted) {
      shooter.stop();
    }
  }

  /*
  1. identify target/prepare vision
  2.
    - drive lock on target
    - startup shooter
  3. input balls to shooter from transporter by trigger
   */
  public Command shoot(BooleanSupplier trigger, Drivetrain drive) {
    Command dtpid = drive.goToAngle(vision::getYawOffset, 0);
    Command visionControl =
        new StartEndCommand(
            () -> vision.goTo(Vision.Mode.TARGET, Vision.CameraPosition.UP),
            () -> vision.goTo(Vision.Mode.DRIVER, Vision.CameraPosition.FORWARD),
            vision);
    Command shooterpid = new ShootCommand(trigger, drive::isAtTargetAngle);

    return new ParallelCommandGroup(dtpid, visionControl, shooterpid);
  }

  public Command dummyShoot(BooleanSupplier trigger) {
    Command visionControl =
        new StartEndCommand(
            () -> vision.goTo(Vision.Mode.TARGET, Vision.CameraPosition.UP),
            () -> vision.goTo(Vision.Mode.DRIVER, Vision.CameraPosition.FORWARD),
            vision);
    Command shooterpid = new ShootCommand(trigger, () -> true);

    return new ParallelCommandGroup(visionControl, shooterpid);
  }
}
