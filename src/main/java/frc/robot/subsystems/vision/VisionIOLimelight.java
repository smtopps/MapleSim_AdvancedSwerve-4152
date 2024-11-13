// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

  @Override
  public void updateInputs(VisionInputs inputs, Pose2d pose) {
    LimelightHelpers.SetRobotOrientation(
        "limelight-shooter", pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
    if (estimate != null && estimate.pose != new Pose2d()) {
      inputs.avgTagArea = estimate.avgTagArea;
      inputs.avgTagDist = estimate.avgTagDist;
      inputs.latency = estimate.latency;
      inputs.pose = estimate.pose;
      inputs.tagCount = estimate.tagCount;
      inputs.tagSpan = estimate.tagSpan;
      inputs.timestampSeconds = estimate.timestampSeconds;
      RawFiducial[] fiducials = estimate.rawFiducials;
      Pose3d[] poses = new Pose3d[fiducials.length];
      for (int i = 0; i < fiducials.length; i++) {
        RawFiducial fiducial = fiducials[i];
        poses[i] = ShooterConstants.aprilTags.getTagPose(fiducial.id).get();
      }
      inputs.tagPoses = poses;
    } else {
      inputs.avgTagArea = 0.0;
      inputs.avgTagDist = 0.0;
      inputs.latency = 0.0;
      inputs.pose = null;
      inputs.tagCount = 0;
      inputs.tagSpan = 0.0;
      inputs.timestampSeconds = 0.0;
      inputs.tagPoses = null;
    }
  }
}
