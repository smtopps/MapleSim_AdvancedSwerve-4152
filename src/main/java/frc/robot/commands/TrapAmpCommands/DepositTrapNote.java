// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorTrap.ElevatorConstants;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;
import frc.robot.subsystems.ElevatorTrap.TrapConstants;

public class DepositTrapNote extends Command {
  private final ElevatorTrap elevatorTrap;
  private double scoreTime;
  /** Creates a new RunTrap. */
  public DepositTrapNote(ElevatorTrap elevatorTrap) {
    this.elevatorTrap = elevatorTrap;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorTrap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorTrap.setTrapSpeed(TrapConstants.trapScoreSpeed);
    elevatorTrap.setElevatorPosition(ElevatorConstants.trapPosition);
    scoreTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorTrap.stopTrapMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - scoreTime > TrapConstants.trapScoreTime) {
      return true;
    } else {
      return false;
    }
  }
}
