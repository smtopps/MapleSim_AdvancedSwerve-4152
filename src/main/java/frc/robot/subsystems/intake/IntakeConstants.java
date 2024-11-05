// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

/** Add your docs here. */
public class IntakeConstants {
  public static final double rotationGearRatio = 45.7143;
  public static final Slot0Configs rotationSlot0Configs =
      new Slot0Configs()
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withKG(0.3)
          .withKS(0.1) // output to overcome static friction (output)
          .withKV(5.08) // output per unit of target velocity (output/rps)
          .withKA(0.03) // output per unit of target acceleration (output/(rps/s))
          .withKP(50) // output per unit of error in position (output/rotation) 25
          .withKI(0.0) // output per unit of integrated error in position (output/(rotation*s))
          .withKD(0.005); // output per unit of error in velocity (output/rps)
  public static final CurrentLimitsConfigs rotationCurrentLimits =
      new CurrentLimitsConfigs().withStatorCurrentLimit(30.0).withStatorCurrentLimitEnable(true);
  public static final VoltageConfigs rotationVoltageConfigs =
      new VoltageConfigs().withPeakForwardVoltage(10.0).withPeakReverseVoltage(10.0);
  public static final FeedbackConfigs rotationFeedbackConfigs =
      new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(rotationGearRatio);
  public static final MotionMagicConfigs rotationMotionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(
              5.0) // controls acceleration and deceleration rates during the beginning and end of
          // motion 3.5
          .withMotionMagicCruiseVelocity(
              2.0) // peak velocity of the profile; set to 0 to target the system’s max velocity
          .withMotionMagicExpo_kA(
              0.353) // voltage required to apply a given acceleration, in V/(rps/s)
          .withMotionMagicExpo_kV(5.17) // voltage required to maintain a given velocity, in V/rps
          .withMotionMagicJerk(0.0); // controls jerk, which is the derivative of acceleration
  public static final SoftwareLimitSwitchConfigs rotationSoftwareLimitSwitchConfigs =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(0.448 + 0.05)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(-0.132);

  public static final double stowedPosition = 0.448;
  public static final double shootPosition = 0.448;
  public static final double floorPosition = -0.1; // -0.112
  public static final double outakePosition = 0.37;
  public static final double sourcePosition = 0.448;
  public static final double ampPosition = 0.2; // 0.238

  public static final double positionError = 0.02; // made larger for school visits 0.01
  // Can assign later if need be
  public static final double floorSpeed = 0.75;
  public static final double stallSpeed = 0;
  public static final double shootSpeed = -1.0;
  public static final double ampSpeed = -5.0; // -0.355
  public static final double trapSpeed = -0.5;
  public static final double shootTrapSpeed = -0.70;
  public static final double outakeSpeed = -1.0;
  public static final double handoffAmpSpeed = -1.0;

  public static final double stallTriggerTime = 0.05;
  public static final double stallRPM = 0.05;
  public static final double fastStallRPM = 3700;
  public static final int rotationMotorID = 22;
  public static final int rollerMotorID = 23;
  public static final int rollerMotorCurrentLimit = 60;
}
