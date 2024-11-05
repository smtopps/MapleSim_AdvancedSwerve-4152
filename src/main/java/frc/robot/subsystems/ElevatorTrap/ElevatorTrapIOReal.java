// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorTrap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorTrapIOReal implements ElevatorTrapIO {
  private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.leftElevatorMotorID);
  private final CANSparkFlex trapMotor =
      new CANSparkFlex(TrapConstants.trapMotorID, MotorType.kBrushless);

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

  @Override
  public void updateInputs(ElevatorTrapInputs inputs) {
    inputs.elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();

    double elevatorRotations =
        MathUtil.inverseInterpolate(0.0, 2.32, elevatorMotor.getPosition().getValueAsDouble());
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
  public void configureElevatorMotor() {
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ElevatorConstants.elevatorSlot0Configs;
    talonfxConfigs.CurrentLimits = ElevatorConstants.elevatorCurrentLimits;
    talonfxConfigs.Voltage = ElevatorConstants.elevatorVoltageConfigs;
    talonfxConfigs.Feedback = ElevatorConstants.elevatorFeedbackConfigs;
    talonfxConfigs.MotionMagic = ElevatorConstants.elevatorMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = ElevatorConstants.elevatorSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    elevatorMotor.getConfigurator().apply(talonfxConfigs);
    elevatorMotor.setInverted(true);
    elevatorMotor.setPosition(0.0);
  }

  @Override
  public void setElevatorPosition(double position) {
    elevatorMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  @Override
  public void moveElevator(double power) {
    elevatorMotor.set(-power);
  }

  @Override
  public void stopElevator() {
    elevatorMotor.stopMotor();
  }

  public void configureTrapMotor() {
    trapMotor.restoreFactoryDefaults();
    trapMotor.setIdleMode(IdleMode.kBrake);
    trapMotor.setSmartCurrentLimit(TrapConstants.trapMotorCurrentLimit);
    trapMotor.stopMotor();
  }

  @Override
  public void setTrapSpeed(double speed) {
    trapMotor.set(speed);
  }

  @Override
  public void stopTrapMotor() {
    trapMotor.stopMotor();
  }
}
