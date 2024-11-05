// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  class ShooterInputs {
    public double leftVelocityRPS = 0.0;
    public double rightVelocityRPS = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterInputs inputs) {}

  default void configureShooterMotors() {}

  default void setShooterSpeeds(double RPS, double spinFactor) {}

  default void stopShooter() {}
}
