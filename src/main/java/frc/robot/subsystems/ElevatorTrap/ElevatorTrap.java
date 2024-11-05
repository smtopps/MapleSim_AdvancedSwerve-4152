// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorTrap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorTrap extends SubsystemBase {
  private final ElevatorTrapIO io;
  private ElevatorTrapInputsAutoLogged inputs;

  public ElevatorTrap(ElevatorTrapIO io) {
    this.io = io;
    this.inputs = new ElevatorTrapInputsAutoLogged();

    io.configureElevatorMotor();
    io.configureTrapMotor();
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("ElevatorTrap", inputs);
  }

  public boolean isElevatorAtPosition(double position) {
    double error = Math.abs(position - inputs.elevatorPosition);
    if (error < ElevatorConstants.positionError) {
      return true;
    } else {
      return false;
    }
  }

  public double getElevatorPosition() {
    return inputs.elevatorPosition;
  }

  public void moveElevator(double power) {
    io.moveElevator(power);
  }

  public void stopElevator() {
    io.stopElevator();
  }

  public void setElevatorPosition(double position) {
    io.setElevatorPosition(position);
  }

  public void setTrapSpeed(double speed) {
    io.setTrapSpeed(speed);
  }

  public void stopTrapMotor() {
    io.stopTrapMotor();
  }
}
