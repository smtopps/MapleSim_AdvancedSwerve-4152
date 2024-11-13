// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIOPhotonSim implements VisionIO {
  private final AbstractDriveTrainSimulation driveTrain;
  // The PhotonCamera used in the real robot code.
  private final PhotonCamera camera = new PhotonCamera("shooter");
  // The simulation of this camera. Its values used in real robot code will be updated.
  PhotonCameraSim cameraSim;

  private final VisionSystemSim visionSim;
  AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
  // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
  Translation3d robotToCameraTrl = new Translation3d(-0.2, 0, 0.6);
  // and pitched 38 degrees up and 180 deg around
  Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-42), Math.toRadians(180));
  Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

  public VisionIOPhotonSim(AbstractDriveTrainSimulation driveTrain) {
    this.driveTrain = driveTrain;
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(tagLayout);

    SimCameraProperties cameraProperties = new SimCameraProperties();
    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(97.7));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProperties.setCalibError(0.3, 0.10);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProperties.setFPS(15);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProperties.setAvgLatencyMs(20);
    cameraProperties.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);

    // Enable the raw and processed streams. These are enabled by default.
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(false);
  }

  @Override
  public void updateInputs(VisionInputs inputs, Pose2d pose) {
    // Update with the simulated drivetrain pose. This should be called every loop in simulation.
    Pose2d simulatedPose = driveTrain.getSimulatedDriveTrainPose();
    if (simulatedPose != null) {
      visionSim.update(simulatedPose);
    }
    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");

    writeToTable(camera.getLatestResult(), shooterTable);

    LimelightHelpers.PoseEstimate estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
    if (estimate != null) {
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

  private void writeToTable(PhotonPipelineResult result, NetworkTable table) {
    // Write to limelight table
    if (result.getMultiTagResult().isPresent()) {
      Transform3d best =
          result.getMultiTagResult().get().estimatedPose.best.plus(robotToCamera.inverse());
      Pose2d fieldToCamera =
          new Pose2d(best.getTranslation().toTranslation2d(), best.getRotation().toRotation2d());
      List<Double> pose_data =
          new ArrayList<>(
              Arrays.asList(
                  best.getX(), // 0: X
                  best.getY(), // 1: Y
                  best.getZ(), // 2: Z
                  0.0, // 3: roll
                  0.0, // 4: pitch
                  fieldToCamera.getRotation().getDegrees(), // 5: yaw
                  result.metadata.getLatencyMillis(), // 6: latency ms
                  (double) result.targets.size(), // 7: tag count
                  0.0, // 8: tag span
                  0.0, // 9: tag dist
                  result.getBestTarget().getArea() // 10: tag area
                  ));
      // Add RawFiducials
      // This is super inefficient but it's sim only, who cares.
      for (var target : result.targets) {
        pose_data.add((double) target.getFiducialId()); // 0: id
        pose_data.add(target.getYaw()); // 1: txnc
        pose_data.add(target.getPitch()); // 2: tync
        pose_data.add(0.0); // 3: ta
        pose_data.add(0.0); // 4: distToCamera
        pose_data.add(0.0); // 5: distToRobot
        pose_data.add(0.5); // 6: ambiguity
      }

      table
          .getEntry("botpose_wpiblue")
          .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
      table
          .getEntry("botpose_orb_wpiblue")
          .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
    } else {
        List<Double> pose_data =
            new ArrayList<>(
                Arrays.asList(
                    0.0, // 0: X
                    0.0, // 1: Y
                    0.0, // 2: Z
                    0.0, // 3: roll
                    0.0, // 4: pitch
                    0.0, // 5: yaw
                    0.0, // 6: latency ms
                    0.0, // 7: tag count
                    0.0, // 8: tag span
                    0.0, // 9: tag dist
                    0.0 // 10: tag area
                    ));
        // Add RawFiducials
        // This is super inefficient but it's sim only, who cares.
        pose_data.add((double) 0.0); // 0: id
        pose_data.add(0.0); // 1: txnc
        pose_data.add(0.0); // 2: tync
        pose_data.add(0.0); // 3: ta
        pose_data.add(0.0); // 4: distToCamera
        pose_data.add(0.0); // 5: distToRobot
        pose_data.add(0.0); // 6: ambiguity

        table
            .getEntry("botpose_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        table
            .getEntry("botpose_orb_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
      }

      table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
      table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
  }
}
