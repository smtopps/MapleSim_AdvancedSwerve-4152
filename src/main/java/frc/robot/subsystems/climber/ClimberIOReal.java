// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  private TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotorID);

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

  @Override
  public void updateInputs(ClimberInputs inputs) {
    double climberPosition =
        MathUtil.inverseInterpolate(0.0, 3.69, climberMotor.getPosition().getValueAsDouble());
    double climberRotatoins = MathUtil.interpolate(0.0, 73.6, climberPosition);
    inputs.climberPose =
        new Pose3d(
            -0.223638,
            0,
            0.642606,
            new Rotation3d(0, -Units.degreesToRadians(climberRotatoins), 0));

    inputs.climberPosition = climberMotor.getPosition().getValueAsDouble();
  }

  public void moveClimber(double power) {
    climberMotor.set(power);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public void setClimberPosition(double position) {
    climberMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  public void configureMotorsControllers() {
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ClimberConstants.climberSlot0Configs;
    talonfxConfigs.CurrentLimits = ClimberConstants.climberCurrentLimits;
    talonfxConfigs.Voltage = ClimberConstants.climberVoltageConfigs;
    talonfxConfigs.Feedback = ClimberConstants.climberFeedbackConfigs;
    talonfxConfigs.MotionMagic = ClimberConstants.climberMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = ClimberConstants.climberSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor.getConfigurator().apply(talonfxConfigs);
    climberMotor.setPosition(0.0);
  }

  public void resetEncoder(boolean stop) {
    if (stop) {
      climberMotor.stopMotor();
    }
    climberMotor.setPosition(0.0);
  }
}
