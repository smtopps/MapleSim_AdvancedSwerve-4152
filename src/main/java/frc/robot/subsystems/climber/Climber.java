// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private ClimberInputsAutoLogged inputs;

  public Climber(ClimberIO io) {
    this.io = io;
    this.inputs = new ClimberInputsAutoLogged();

    io.configureMotorControllers();
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void moveClimber(double power) {
    io.moveClimber(power);
  }

  public void stopClimber() {
    io.stopClimber();
  }

  public void setClimberPosition(double position) {
    io.setClimberPosition(position);
  }

  public double getClimberPosition() {
    return inputs.climberPosition;
  }

  public void resetEncoder(boolean stop) {
    io.resetEncoder(stop);
  }

  public boolean isClimberAtPosition(double position) {
    double error = Math.abs(position - inputs.climberPosition);
    if (error < ClimberConstants.positionError) {
      return true;
    } else {
      return false;
    }
  }
}
