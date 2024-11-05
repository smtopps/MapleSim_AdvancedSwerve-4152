// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;
import java.util.function.DoubleSupplier;

public class TrapManual extends Command {
  private final ElevatorTrap elevatorTrap;
  private final DoubleSupplier trapSpeed;
  /** Creates a new RunTrap. */
  public TrapManual(ElevatorTrap elevatorTrap, DoubleSupplier trapSpeed) {
    this.elevatorTrap = elevatorTrap;
    this.trapSpeed = trapSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorTrap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorTrap.setTrapSpeed(trapSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorTrap.setTrapSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
