// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorTrap.ElevatorConstants;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class AutoShootDeflect extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private final ElevatorTrap elevatorTrap;
  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new ShootDeflect. */
  public AutoShootDeflect(Shooter shooter, Intake intake, ElevatorTrap elevatorTrap) {
    this.shooter = shooter;
    this.intake = intake;
    this.elevatorTrap = elevatorTrap;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake, elevatorTrap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(ShooterConstants.deflectRPS, 0.0);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    elevatorTrap.setElevatorPosition(ElevatorConstants.ShootDeflect);
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isShooterAtSpeed(ShooterConstants.deflectRPS, 0.0)
        && elevatorTrap.isElevatorAtPosition(ElevatorConstants.ShootDeflect)
        && intake.isIntakeAtPosition(IntakeConstants.shootPosition)) {
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
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    elevatorTrap.setElevatorPosition(ElevatorConstants.elevatorStowedPosition);
    shooter.setShooterSpeeds(ShooterConstants.shootingRPS, ShooterConstants.spinFactor);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
