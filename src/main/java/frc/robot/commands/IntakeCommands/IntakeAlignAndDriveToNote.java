// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class IntakeAlignAndDriveToNote extends Command {
  private final Intake intake;
  private final Drive drivetrain;
  private final DoubleSupplier triggerTranslation;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;

  private final double triggerThreshold = 0.5;
  private final double topSpeed = 2.0;

  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);

  /** Creates a new Intake. */
  public IntakeAlignAndDriveToNote(
      Intake intake,
      Drive drivetrain,
      DoubleSupplier triggerTranslation,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotation) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.triggerTranslation = triggerTranslation;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);
    LimelightHelpers.setPipelineIndex("limelight-intake", 0);
    yawPIDController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-intake")) {
      double yawSpeed = yawPIDController.calculate(LimelightHelpers.getTX("limelight-intake"));
      double xSpeed =
          MathUtil.inverseInterpolate(triggerThreshold, 1.0, triggerTranslation.getAsDouble());
      // xSpeed = Math.pow(xSpeed, 3.0);
      xSpeed = MathUtil.interpolate(0.25, topSpeed, xSpeed);
      if (triggerTranslation.getAsDouble() >= triggerThreshold
          && intake.isIntakeAtPosition(IntakeConstants.floorPosition)) {
        drivetrain.runVelocity(new ChassisSpeeds(xSpeed, translationY.getAsDouble(), yawSpeed));
      } else {
        drivetrain.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                yawSpeed,
                drivetrain.getRotation()));
      }
    } else {
      // drivetrain.setControl(fieldRequest.withRotationalRate(rotation.getAsDouble()).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
      drivetrain.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationX.getAsDouble(),
              translationY.getAsDouble(),
              rotation.getAsDouble(),
              drivetrain.getRotation()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
