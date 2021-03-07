package io.excaliburfrc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.vision.Limelight;

import static io.excaliburfrc.robot.Constants.LIMELIGHT_FWD;
import static io.excaliburfrc.robot.Constants.LIMELIGHT_REV;

public class Vision extends SubsystemBase {
    private Limelight limelight;
    private DoubleSolenoid piston;

    public Vision() {
        limelight = new Limelight();
        piston = new DoubleSolenoid(LIMELIGHT_FWD, LIMELIGHT_REV);
    }

    public void raise() {
        piston.set(DoubleSolenoid.Value.kReverse);
    }

    public void lower() {
        piston.set(DoubleSolenoid.Value.kForward);
    }

    public double getDistance() {
        //  TODO : calebrate
        return -1;
    }
//    HasValidTargets("tv"),
}
