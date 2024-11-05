// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {
  private double climberPosition;
  private double climberPower;
  private ElevatorSim climberArmSim;

  public ClimberIOSim() {
    climberArmSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            ClimberConstants.climberGearRatio,
            0.1,
            0.375,
            -0.5,
            3.69,
            false,
            0.0);
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    climberArmSim.setInputVoltage(climberPower);
    climberArmSim.update(0.02);

    double climberPosition =
        MathUtil.inverseInterpolate(0.0, 3.69, climberArmSim.getPositionMeters());
    double climberRotatoins = MathUtil.interpolate(0.0, 73.6, climberPosition);
    inputs.climberPose =
        new Pose3d(
            -0.223638,
            0,
            0.642606,
            new Rotation3d(0, -Units.degreesToRadians(climberRotatoins), 0));

    inputs.climberPosition = climberArmSim.getPositionMeters();
  }

  public void moveClimber(double power) {
    climberPower = power * 12;
  }

  public void stopClimber() {
    climberPower = 0.0;
  }

  public void setClimberPosition(double position) {}

  public void configureMotorsControllers() {}

  public void resetEncoder(boolean stop) {}
}
