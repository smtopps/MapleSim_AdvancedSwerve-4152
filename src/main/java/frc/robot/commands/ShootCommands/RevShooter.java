// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RevShooter extends Command {
  private final Drive drivetrain;
  private final Shooter shooter;

  private double distanceToTarget;
  private int targetTag;
  private Pose2d targetPose;
  private boolean previousState;
  private boolean currentState;
  /** Creates a new RevShooter. */
  public RevShooter(Drive drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousState = false;
    currentState = false;
    targetTag = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    if (distanceToTarget < Units.inchesToMeters(160)) {
      currentState = true;
    } else {
      currentState = false;
    }

    if (currentState != previousState && currentState == true) {
      shooter.setShooterSpeeds(ShooterConstants.ampHandoffRPS, 0.0);
      previousState = true;
    } else if (currentState != previousState && currentState == false) {
      shooter.stopShooter();
      previousState = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    previousState = false;
    currentState = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
