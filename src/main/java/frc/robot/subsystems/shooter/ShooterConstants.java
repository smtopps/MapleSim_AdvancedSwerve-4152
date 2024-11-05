// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Add your docs here. */
public class ShooterConstants {
  public static final double gearRatio = 1.25;
  public static final Slot0Configs shooterSlot0Configs =
      new Slot0Configs()
          .withKS(0.27) // output to overcome static friction (output)
          .withKV(0.1538) // output per unit of target velocity (output/rps)
          .withKP(0.75) // output per unit of error in position (output/rotation)
          .withKI(0.0) // output per unit of integrated error in position (output/(rotation*s))
          .withKD(0.0); // output per unit of error in velocity (output/rps)
  public static final CurrentLimitsConfigs shooterCurrentLimits =
      new CurrentLimitsConfigs().withStatorCurrentLimit(45.0).withStatorCurrentLimitEnable(true);
  public static final VoltageConfigs shooterVoltageConfigs =
      new VoltageConfigs().withPeakForwardVoltage(11.0).withPeakReverseVoltage(11.0);
  public static final FeedbackConfigs shooterFeedbackConfigs =
      new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(gearRatio);

  public static final double shootingRPS = 70;
  public static final double spinFactor = 0.05;
  public static final double speedThreshold = 1.5;
  public static final double deflectRPS = 67;
  public static final double trapHandoffRPS = 8.4;
  public static final double ampHandoffRPS = 18.84;

  public static final double shootPosition = 3.30; // 2.95

  public static final double xSetPoint = 0;
  public static final double ySetPoint = 0;

  public static final double shootingAngle = 38; // to be used later for projectile profiling.

  public static final int leftShooterMotorID = 24;
  public static final int rightShooterMotorID = 25;

  public static AprilTagFieldLayout aprilTags;
}
