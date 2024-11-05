// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorTrap;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorTrapIO {
  @AutoLog
  class ElevatorTrapInputs {
    public Pose3d elevatorStage2Pose;
    public Pose3d elevatorStage3Pose;
    public double elevatorPosition;
  }

  default void moveElevator(double power) {}

  default void stopElevator() {}

  default void setElevatorPosition(double position) {}

  default void configureElevatorMotor() {}

  default void configureTrapMotor() {}

  default void setTrapSpeed(double speed) {}

  default void stopTrapMotor() {}

  public default void updateInputs(ElevatorTrapInputs inputs) {}
}
