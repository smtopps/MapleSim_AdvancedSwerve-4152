// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the NewIntake subsystem */
public interface IntakeIO {
  /** stores the inputs data of intake */
  @AutoLog
  class IntakeInputs {
    public boolean noteDetected;
    public Pose3d intakePose;
    public double intakeRotation;
    public double intakeVoltageIntegralSinceNoteTaken;
    public double rollerSpeed;
  }

  default void configureRotationMotor() {}

  default void configureRollerMotor() {}

  default void setIntakeRollerCurrentLimit(int current) {}

  default void setIntakePosition(double position) {}

  default void setRollerSpeed(double speed) {}

  default void stopRotation() {}

  default void stopRoller() {}

  /** update the sensor inputs */
  void updateInputs(IntakeInputs inputs);
}
