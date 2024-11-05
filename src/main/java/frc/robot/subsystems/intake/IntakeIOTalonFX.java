// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rotationMotor = new TalonFX(IntakeConstants.rotationMotorID);
  private final CANSparkFlex rollerMotor =
      new CANSparkFlex(IntakeConstants.rollerMotorID, MotorType.kBrushless);
  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // detect the note with the beam breaker and store it in "inputs"
    // if the note is present, the beam breaker is blocked and will return "false"
    inputs.noteDetected = true;
    inputs.intakePose =
        new Pose3d(
            0.32004,
            0,
            0.280616,
            new Rotation3d(0, -rotationMotor.getPosition().getValueAsDouble() - 0.448, 0));
    inputs.intakeRotation = rotationMotor.getPosition().getValueAsDouble();
  }

  public void configureRotationMotor() {
    rotationMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = IntakeConstants.rotationSlot0Configs;
    talonfxConfigs.CurrentLimits = IntakeConstants.rotationCurrentLimits;
    talonfxConfigs.Voltage = IntakeConstants.rotationVoltageConfigs;
    talonfxConfigs.Feedback = IntakeConstants.rotationFeedbackConfigs;
    talonfxConfigs.MotionMagic = IntakeConstants.rotationMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = IntakeConstants.rotationSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rotationMotor.getConfigurator().apply(talonfxConfigs);
    rotationMotor.setPosition(IntakeConstants.stowedPosition);
    setIntakePosition(IntakeConstants.stowedPosition);
  }

  public void configureRollerMotor() {
    rollerMotor.restoreFactoryDefaults();
    rollerMotor.setIdleMode(IdleMode.kCoast);
    rollerMotor.setSmartCurrentLimit(IntakeConstants.rollerMotorCurrentLimit); // 55
    rollerMotor.setInverted(true);
    rollerMotor.set(IntakeConstants.stallSpeed);
  }

  public void setIntakeRollerCurrentLimit(int current) {
    rollerMotor.setSmartCurrentLimit(current);
  }

  public void setIntakePosition(double position) {
    rotationMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRotation() {
    rotationMotor.stopMotor();
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
