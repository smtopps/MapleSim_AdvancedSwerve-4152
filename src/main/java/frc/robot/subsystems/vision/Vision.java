// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Vision extends SubsystemBase {
  private final VisionIO io;
  private Drive drivetrain;
  private VisionInputsAutoLogged inputs;

  public Vision(VisionIO io, Drive drivetrain) {
    this.io = io;
    this.drivetrain = drivetrain;
    this.inputs = new VisionInputsAutoLogged();
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs, drivetrain.getPose());
    Logger.processInputs("Vision", inputs);
    if (inputs.pose != null
        && inputs.tagCount > 0
        && Math.toDegrees(drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond) < 720) {
      double confidence;
      confidence = 2.0 / inputs.tagCount;
      confidence = confidence + inputs.avgTagDist * 0.5;
      Logger.recordOutput("confidence", confidence);
      drivetrain.addVisionMeasurement(
          inputs.pose, inputs.timestampSeconds, VecBuilder.fill(confidence, confidence, 9999999));
    }
  }
}
