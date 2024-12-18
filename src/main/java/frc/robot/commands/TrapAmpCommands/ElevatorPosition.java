// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;

public class ElevatorPosition extends Command {
  private final ElevatorTrap elevatorTrap;
  private double desiredPosition;
  /** Creates a new ElevatorPosition. */
  public ElevatorPosition(ElevatorTrap elevatorTrap, double desiredPosition) {
    this.elevatorTrap = elevatorTrap;
    this.desiredPosition = desiredPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorTrap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorTrap.setElevatorPosition(desiredPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorTrap.isElevatorAtPosition(desiredPosition);
  }
}
