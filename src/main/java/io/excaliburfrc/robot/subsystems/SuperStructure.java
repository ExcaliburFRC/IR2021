package io.excaliburfrc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SuperStructure extends SubsystemBase {
  public final Intake intake = new Intake();
  public final Transporter transporter = new Transporter();
  public final Shooter shooter = new Shooter();
  public final Vision vision = new Vision();

  public void init() {
    intake.raise();
    stop();
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
    private final BooleanSupplier eject;
    private final DoubleSupplier distanceSource;
    private final double target;

    public ShootCommand(
        BooleanSupplier trigger,
        BooleanSupplier isDriveLocked,
        BooleanSupplier eject,
        DoubleSupplier distanceSource) {
      this.distanceSource = distanceSource;
      addRequirements(shooter, transporter, vision);
      this.eject = eject;
      this.trigger = trigger;
      this.isDriveLocked = isDriveLocked;
      target = 0;
    }

    public ShootCommand(
          BooleanSupplier trigger,
          BooleanSupplier isDriveLocked,
          BooleanSupplier eject,
          double target) {
      addRequirements(shooter, transporter, vision);
      this.distanceSource = null;
      this.target = target;
      this.eject = eject;
      this.trigger = trigger;
      this.isDriveLocked = isDriveLocked;
    }

    @Override
    public void initialize() {
      if (distanceSource == null) {
        shooter.start(target);
      } else {
        shooter.startToVisionDistance(distanceSource.getAsDouble());
      }
    }

    @Override
    public void execute() {
      LEDs.INSTANCE.setMode(LedMode.RED);
      final boolean isDriveLockedAsBoolean = isDriveLocked.getAsBoolean();
      SmartDashboard.putBoolean("isDriveAligned", isDriveLockedAsBoolean);
      if (trigger.getAsBoolean() && isDriveLockedAsBoolean && shooter.isAtTargetVelocity()) {
        transporter.activate(Transporter.Mode.SHOOT);
      } else if (eject.getAsBoolean()) {
        transporter.activate(Transporter.Mode.OUT);
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
  public Command shoot(BooleanSupplier trigger, BooleanSupplier eject, Drivetrain drive) {
    Command dtpid = drive.goToAngle(vision::getYawOffset, 0);
    Command shooterpid =
        new ShootCommand(trigger, drive::isAtTargetAngle, eject, vision::getDistance);

    var leds = new InstantCommand(() -> LEDs.INSTANCE.setMode(LedMode.GREEN));
    return new ParallelCommandGroup(dtpid, shooterpid, leds);
  }

  public Command dummyShoot(BooleanSupplier trigger, BooleanSupplier eject) {
    Command shooterpid = new ShootCommand(trigger, () -> true, eject, 80);

    var leds = new InstantCommand(() -> LEDs.INSTANCE.setMode(LedMode.GREEN));
    return new ParallelCommandGroup(shooterpid, leds);
  }
}
