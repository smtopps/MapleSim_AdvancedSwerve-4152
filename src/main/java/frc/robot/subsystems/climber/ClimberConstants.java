package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ClimberConstants {
  public static final double climberGearRatio = 45.0;
  public static final Slot0Configs climberSlot0Configs =
      new Slot0Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKS(0.165) // output to overcome static friction (output)
          .withKV(5) // output per unit of target velocity (output/rps)
          .withKA(0.475) // output per unit of target acceleration (output/(rps/s))
          .withKP(5) // output per unit of error in position (output/rotation)
          .withKI(0.0) // output per unit of integrated error in position (output/(rotation*s))
          .withKD(0.0); // output per unit of error in velocity (output/rps)
  public static final CurrentLimitsConfigs climberCurrentLimits =
      new CurrentLimitsConfigs().withStatorCurrentLimit(50.0).withStatorCurrentLimitEnable(true);
  public static final VoltageConfigs climberVoltageConfigs =
      new VoltageConfigs().withPeakForwardVoltage(10.0).withPeakReverseVoltage(10.0);
  public static final FeedbackConfigs climberFeedbackConfigs =
      new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(climberGearRatio);
  public static final MotionMagicConfigs climberMotionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(
              6.0) // controls acceleration and deceleration rates during the beginning and end of
          // motion
          .withMotionMagicCruiseVelocity(
              3.0) // peak velocity of the profile; set to 0 to target the system’s max velocity
          .withMotionMagicExpo_kA(
              0.0) // voltage required to apply a given acceleration, in V/(rps/s)
          .withMotionMagicExpo_kV(0.0) // voltage required to maintain a given velocity, in V/rps
          .withMotionMagicJerk(0.0); // controls jerk, which is the derivative of acceleration
  public static final SoftwareLimitSwitchConfigs climberSoftwareLimitSwitchConfigs =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(3.69)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(-0.5);
  public static final int climberMotorID = 27;

  public static final double midCamClearPosition = 1.0; // 0.546783
  public static final double climbingPosition = 3.6;
  public static final double positionError = 0.05;
  public static final double readyToClimbPosition = 3.0;
}
