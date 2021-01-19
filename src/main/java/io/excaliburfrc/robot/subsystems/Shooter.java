package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax shooterMotor;

    public Shooter() {
        shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
        shooterMotor.setIdleMode(IdleMode.kCoast);
    }

    public void start(ShooterSpeed speed) {
        shooterMotor.getPIDController().setReference(speed.rpm, ControlType.kVelocity);
    }

    public void stop() {
        shooterMotor.stopMotor();
    }


    public enum ShooterSpeed {HIGH(6000), LOW(2000);
        public double rpm;
        ShooterSpeed(double i) {
            this.rpm = i;
        }
    }
}