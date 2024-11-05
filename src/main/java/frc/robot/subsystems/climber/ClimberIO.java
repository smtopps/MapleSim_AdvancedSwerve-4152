// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {
  @AutoLog
  class ClimberInputs {
    public Pose3d climberPose;
    public double climberPosition;
  }

  default void moveClimber(double power) {}

  default void stopClimber() {}

  default void setClimberPosition(double position) {}

  default void configureMotorControllers() {}

  default void resetEncoder(boolean stop) {}

  public default void updateInputs(ClimberInputs inputs) {}
}
