// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorTrap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorTrapIOSim implements ElevatorTrapIO {

  private double elevatorPosition;
  private ElevatorSim sim;
  private PIDController positionController;

  public ElevatorTrapIOSim() {
    sim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            ElevatorConstants.elevatorGearRatio,
            20,
            Units.inchesToMeters(1.751 / 2),
            0,
            Units.inchesToMeters(12.5),
            false,
            0);
    positionController = new PIDController(100.0, 0, 0);
  }

  @Override
  public void updateInputs(ElevatorTrapInputs inputs) {
    double circumference = Units.inchesToMeters(Math.PI * 1.751);
    positionController.setSetpoint(elevatorPosition * circumference);
    sim.setInputVoltage(positionController.calculate(sim.getPositionMeters()));
    sim.update(0.02);

    inputs.elevatorPosition = sim.getPositionMeters() / circumference;

    double elevatorRotations =
        MathUtil.inverseInterpolate(0.0, 2.32, sim.getPositionMeters() / circumference);
    double elevator2Positions = MathUtil.interpolate(0.0, 0.3175, elevatorRotations);
    double elevator3Positions = MathUtil.interpolate(0.0, 0.635, elevatorRotations);
    double xTranslation2 = Math.sin(Units.degreesToRadians(22)) * elevator2Positions;
    double zTranslation2 = Math.cos(Units.degreesToRadians(22)) * elevator2Positions;
    double xTranslation3 = Math.sin(Units.degreesToRadians(22)) * elevator3Positions;
    double zTranslation3 = Math.cos(Units.degreesToRadians(22)) * elevator3Positions;

    inputs.elevatorStage2Pose =
        new Pose3d(-xTranslation2 - 0.139299, 0.0, zTranslation2 + 0.134462, new Rotation3d());
    inputs.elevatorStage3Pose =
        new Pose3d(-xTranslation3 - 0.148814, 0.0, zTranslation3 + 0.158012, new Rotation3d());
  }

  @Override
  public void moveElevator(double power) {}

  @Override
  public void stopElevator() {}

  @Override
  public void setElevatorPosition(double position) {
    elevatorPosition = position;
  }

  @Override
  public void configureElevatorMotor() {
    elevatorPosition = ElevatorConstants.elevatorStowedPosition;
  }
}
