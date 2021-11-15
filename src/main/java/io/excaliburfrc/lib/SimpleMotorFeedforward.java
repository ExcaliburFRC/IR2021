// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package io.excaliburfrc.lib;

import edu.wpi.first.wpilibj.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;

/** A helper class that computes feedforward outputs for a simple permanent-magnet DC motor. */
@SuppressWarnings("MemberName")
public class SimpleMotorFeedforward extends edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward {

  public SimpleMotorFeedforward(double ks, double kv, double ka) {
    super(ks, kv, ka);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity The next velocity setpoint.
   * @param dtSeconds Time between velocity setpoints in seconds.
   * @return The computed feedforward.
   */
  public double calculate(double currentVelocity, double nextVelocity, double dtSeconds) {
    var plant = LinearSystemId.identifyVelocitySystem(this.kv, this.ka);
    var feedforward = new LinearPlantInversionFeedforward<>(plant, dtSeconds);

    var r = Matrix.mat(Nat.N1(), Nat.N1()).fill(currentVelocity);
    var nextR = Matrix.mat(Nat.N1(), Nat.N1()).fill(nextVelocity);

    return ks * Math.signum(currentVelocity) + feedforward.calculate(r, nextR).get(0, 0);
  }

}
