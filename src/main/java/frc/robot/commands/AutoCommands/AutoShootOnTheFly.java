// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class AutoShootOnTheFly extends Command {
  private final Drive drivetrain;
  private final Intake intake;
  private final Shooter shooter;
  private final double speed = ShooterConstants.shootingRPS;
  private final double spin = ShooterConstants.spinFactor;
  private int targetTag;
  private Pose2d targetPose;
  private double distanceToTarget;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new AutoShootOnTheFly. */
  public AutoShootOnTheFly(Drive drivetrain, Shooter shooter, Intake intake) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(speed, spin);
    intake.setIntakeRollerCurrentLimit(100);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    if (intake.isIntakeAtPosition(IntakeConstants.shootPosition)
        && shooter.isShooterAtSpeed(ShooterConstants.shootingRPS, ShooterConstants.spinFactor)
        && distanceToTarget < Units.inchesToMeters(180.0)) {
      intake.setRollerSpeed(IntakeConstants.shootSpeed);
      if (timeStampLock) {
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if (!timeStampLock && Timer.getFPGATimestamp() - shootTime > 0.2) {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setIntakeRollerCurrentLimit(IntakeConstants.rollerMotorCurrentLimit);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
