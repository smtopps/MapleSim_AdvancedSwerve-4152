// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    if (inputs.pose != null && inputs.pose != new Pose2d()) {
      drivetrain.addVisionMeasurement(
          inputs.pose, inputs.timestampSeconds, VecBuilder.fill(0.9, 0.9, 9999999));
    }
  }
}
