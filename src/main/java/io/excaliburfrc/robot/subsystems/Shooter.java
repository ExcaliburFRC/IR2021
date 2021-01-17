package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import static io.excaliburfrc.robot.Constants.DriveConstants.SHOOTER_ID;
import static io.excaliburfrc.robot.Constants.DriveConstants.SHOOTER_SPEED;

public class Shooter extends subsytems  {

    private CANSparkMax sparkMax;

    public Shooter() {
        sparkMax  = new CANSparkMax(SHOOTER_ID,CANSparkMaxLowLevel.MotorType.kBrushless;
    }


    public void shoot() {
        sparkMax.set(SHOOTER_SPEED);
    }



}