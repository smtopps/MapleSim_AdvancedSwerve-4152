// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private ShooterInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterInputsAutoLogged();
    configureShooterMotors();
    ShooterConstants.aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  private void configureShooterMotors() {
    io.configureShooterMotors();
  }

  public void setShooterSpeeds(double RPS, double spinFactor) {
    io.setShooterSpeeds(RPS, spinFactor);
  }

  public void stopShooter() {
    io.stopShooter();
  }

  public boolean isShooterAtSpeed(double RPS, double spinFactor) {
    double shootLeftError = Math.abs(inputs.leftVelocityRPS - (RPS * (1 - spinFactor)));
    double shootRightError = Math.abs(inputs.rightVelocityRPS - (RPS * (1 + spinFactor)));
    if (shootLeftError < ShooterConstants.speedThreshold
        && shootRightError < ShooterConstants.speedThreshold) {
      return true;
    } else {
      return false;
    }
  }
}
