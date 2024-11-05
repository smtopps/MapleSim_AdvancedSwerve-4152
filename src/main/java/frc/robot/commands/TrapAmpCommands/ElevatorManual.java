// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;
import java.util.function.DoubleSupplier;

public class ElevatorManual extends Command {
  private final ElevatorTrap elevatorTrap;
  private final DoubleSupplier movePower;

  /** Creates a new Climb. */
  public ElevatorManual(ElevatorTrap elevatorTrap, DoubleSupplier movePower) {
    this.elevatorTrap = elevatorTrap;
    this.movePower = movePower;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorTrap.moveElevator(movePower.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorTrap.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
