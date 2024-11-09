// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class PIDToPose extends Command {
  private final Drive drivetrain;
  private final Pose2d endPose;
  private final PIDController yawController = new PIDController(0.1, 0, 0);
  private final PIDController xController = new PIDController(3.5, 0, 0.02);
  private final PIDController yController = new PIDController(3.5, 0, 0.02);

  private boolean finished = false;

  /** Creates a new PIDToPose. */
  public PIDToPose(Drive drivetrain, Pose2d endPose) {
    this.drivetrain = drivetrain;
    this.endPose = endPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    yawController.setSetpoint(endPose.getRotation().getDegrees());
    yawController.setTolerance(2.0);
    yawController.enableContinuousInput(-180.0, 180);
    xController.setSetpoint(endPose.getX());
    xController.setTolerance(Units.inchesToMeters(2.0));
    yController.setSetpoint(endPose.getY());
    yController.setTolerance(Units.inchesToMeters(2.0));
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawSpeed = yawController.calculate(drivetrain.getPose().getRotation().getDegrees());
    yawSpeed = MathUtil.clamp(yawSpeed, -3.8, 3.8);
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    if (Math.abs(yawController.getError()) < 5) {
      xSpeed = xController.calculate(drivetrain.getPose().getX());
      xSpeed = MathUtil.clamp(xSpeed, -4.0, 4.0);
      ySpeed = yController.calculate(drivetrain.getPose().getY());
      ySpeed = MathUtil.clamp(ySpeed, -4.0, 4.0);
    }

    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, yawSpeed, drivetrain.getRotation()));

    if (xController.atSetpoint() && yController.atSetpoint()) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    finished = false;
    xController.reset();
    yController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
