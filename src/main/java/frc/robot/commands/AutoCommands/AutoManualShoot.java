// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class AutoManualShoot extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final double speed = ShooterConstants.shootingRPS;
  private final double spin = ShooterConstants.spinFactor;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new ManuelShoot. */
  public AutoManualShoot(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(speed, spin);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isShooterAtSpeed(speed, spin)
        && intake.isIntakeAtPosition(IntakeConstants.shootPosition)) {

      intake.setRollerSpeed(IntakeConstants.shootSpeed);
      if (timeStampLock) {
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if (!timeStampLock && Timer.getFPGATimestamp() - shootTime > 0.4) {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
