// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorTrap.ElevatorConstants;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;
import frc.robot.subsystems.ElevatorTrap.TrapConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class TrapHandoff extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final ElevatorTrap elevatorTrap;
  private double handoffTime;

  /** Creates a new TrapHandoff. */
  public TrapHandoff(Intake intake, Shooter shooter, ElevatorTrap elevatorTrap) {
    this.intake = intake;
    this.shooter = shooter;
    this.elevatorTrap = elevatorTrap;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, elevatorTrap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(ShooterConstants.trapHandoffRPS, 0.0);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    intake.setRollerSpeed(IntakeConstants.trapSpeed);
    elevatorTrap.setElevatorPosition(ElevatorConstants.elevatorHandoffPosition);
    elevatorTrap.setTrapSpeed(TrapConstants.trapHandoffSpeed);
    handoffTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallRPM);
    elevatorTrap.stopTrapMotor();
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - handoffTime > TrapConstants.trapHandoffTime) {
      return true;
    } else {
      return false;
    }
  }
}
