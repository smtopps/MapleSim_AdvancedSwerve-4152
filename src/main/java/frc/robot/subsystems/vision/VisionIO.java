// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  class VisionInputs {
    public double avgTagArea;
    public double avgTagDist;
    public double latency;
    public Pose2d pose;
    public int tagCount;
    public double tagSpan;
    public double timestampSeconds;
    public Pose3d[] tagPoses;
  }

  public default void updateInputs(VisionInputs inputs, Pose2d pose) {}
}
