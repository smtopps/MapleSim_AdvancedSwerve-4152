// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private IntakeInputsAutoLogged inputs;

  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeInputsAutoLogged();
    configureRotationMotor();
    configureRollerMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  private static final Translation2d shooterTranslationOnRobot = new Translation2d(0.1, 0);
  private static final double shooterHeightMeters = 0.33,
      shooterPitchAngleRad = -Math.toRadians(50);

  public void visualizeNoteInIntake(Pose2d robotPose) {
    final Translation2d note2dCenter =
        robotPose
            .getTranslation()
            .plus(shooterTranslationOnRobot.rotateBy(robotPose.getRotation()));
    final Translation3d notePosition =
        new Translation3d(note2dCenter.getX(), note2dCenter.getY() + 0.03, shooterHeightMeters);
    final Rotation3d noteRotation =
        new Rotation3d(
            0, shooterPitchAngleRad, robotPose.getRotation().plus(Rotation2d.k180deg).getRadians());
    Logger.recordOutput(
        "Intake/NoteInIntake",
        inputs.noteDetected
            ? new Pose3d[] {new Pose3d(notePosition, noteRotation)}
            : new Pose3d[0]);
    Logger.recordOutput("Intake/IntakePose", inputs.intakePose);
  }

  public boolean isIntakeAtPosition(double position) {
    double error = Math.abs(position - inputs.intakeRotation);
    if (error < IntakeConstants.positionError) {
      return true;
    } else {
      return false;
    }
  }

  private void configureRotationMotor() {
    io.configureRotationMotor();
  }

  private void configureRollerMotor() {
    io.configureRollerMotor();
  }

  public void setIntakeRollerCurrentLimit(int current) {
    io.setIntakeRollerCurrentLimit(current);
  }

  public void setIntakePosition(double position) {
    io.setIntakePosition(position);
  }

  public void setRollerSpeed(double speed) {
    io.setRollerSpeed(speed);
  }

  public void stopRotation() {
    io.stopRotation();
  }

  public void stopRoller() {
    io.stopRoller();
  }
}
