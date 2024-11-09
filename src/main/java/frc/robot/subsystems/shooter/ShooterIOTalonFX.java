// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {
  private TalonFX shootLeft = new TalonFX(ShooterConstants.leftShooterMotorID);
  private TalonFX shootRight = new TalonFX(ShooterConstants.rightShooterMotorID);
  final VelocityVoltage voltageRequest = new VelocityVoltage(0);
  private double leftSetpoint;
  private double rightSetpoint;

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.leftVelocitySetpoint = leftSetpoint;
    inputs.rightVelocitySetpoint = rightSetpoint;
  }

  public void configShooterMotors() {
    shootLeft.getConfigurator().apply(new TalonFXConfiguration());
    shootRight.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ShooterConstants.shooterSlot0Configs;
    talonfxConfigs.CurrentLimits = ShooterConstants.shooterCurrentLimits;
    talonfxConfigs.Voltage = ShooterConstants.shooterVoltageConfigs;
    talonfxConfigs.Feedback = ShooterConstants.shooterFeedbackConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootLeft.getConfigurator().apply(talonfxConfigs);
    shootRight.getConfigurator().apply(talonfxConfigs);
    shootRight.setInverted(true);
  }

  public void setShooterSpeeds(double RPS, double spinFactor) {
    leftSetpoint = RPS * (1 - spinFactor);
    shootLeft.setControl(voltageRequest.withVelocity(leftSetpoint));
    rightSetpoint = RPS * (1 + spinFactor);
    shootRight.setControl(voltageRequest.withVelocity(rightSetpoint));
  }

  public void stopShooter() {
    shootLeft.stopMotor();
    shootRight.stopMotor();
  }
}
