// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LimelightHelpers;

public class AutoAlignNotes extends Command {
  private final Drive drivetrain;
  private final Intake intake;
  private final PIDController xController;
  /** Creates a new AutoAlign. */
  public AutoAlignNotes(Drive drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

    xController = new PIDController(0.05, 0, 0);
    xController.setSetpoint(0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-intake", 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-intake")) {
      double limelighMeasurement = LimelightHelpers.getTX("limelight-intake");
      PPHolonomicDriveController.overrideRotationFeedback(
          () -> xController.calculate(limelighMeasurement));
      // PPHolonomicDriveController.overrideXFeedback(() ->
      // xController.calculate(limelighMeasurement));
      // limelighMeasurement = MathUtil.inverseInterpolate(-31.25, 31.25, limelighMeasurement);
      // limelighMeasurement = MathUtil.interpolate(-20, 20, limelighMeasurement);
      // double rotation = drivetrain.getPose().getRotation().getDegrees() - limelighMeasurement;
      // Rotation2d rotationTarget = Rotation2d.fromDegrees(rotation);
      // PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(rotationTarget));

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
