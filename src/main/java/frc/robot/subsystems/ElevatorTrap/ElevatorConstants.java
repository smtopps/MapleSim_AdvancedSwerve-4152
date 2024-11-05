package frc.robot.subsystems.ElevatorTrap;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElevatorConstants {
  public static final double elevatorGearRatio = ((68.0 / 18.0) * 5.0);

  public static final Slot0Configs elevatorSlot0Configs =
      new Slot0Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKG(0.2)
          .withKS(0.1) // output to overcome static friction (output)
          .withKV(2.25) // output per unit of target velocity (output/rps)
          .withKA(.03) // output per unit of target acceleration (output/(rps/s))
          .withKP(10) // output per unit of error in position (output/rotation)
          .withKI(0.0) // output per unit of integrated error in position (output/(rotation*s))
          .withKD(0.0); // output per unit of error in velocity (output/rps)
  public static final CurrentLimitsConfigs elevatorCurrentLimits =
      new CurrentLimitsConfigs().withStatorCurrentLimit(40.0).withStatorCurrentLimitEnable(true);
  public static final VoltageConfigs elevatorVoltageConfigs =
      new VoltageConfigs().withPeakForwardVoltage(10.0).withPeakReverseVoltage(10.0);
  public static final FeedbackConfigs elevatorFeedbackConfigs =
      new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(elevatorGearRatio);
  public static final MotionMagicConfigs elevatorMotionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(
              30.0) // controls acceleration and deceleration rates during the beginning and end of
          // motion
          .withMotionMagicCruiseVelocity(
              3.0) // peak velocity of the profile; set to 0 to target the system’s max velocity
          .withMotionMagicExpo_kA(
              0.0) // voltage required to apply a given acceleration, in V/(rps/s)
          .withMotionMagicExpo_kV(0.0) // voltage required to maintain a given velocity, in V/rps
          .withMotionMagicJerk(0.0); // controls jerk, which is the derivative of acceleration
  public static final SoftwareLimitSwitchConfigs elevatorSoftwareLimitSwitchConfigs =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(2.32)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(0.0);
  public static final int leftElevatorMotorID = 26;

  public static final double elevatorStowedPosition = 0.0;
  public static final double elevatorHandoffPosition = 0.5; // 0.6
  public static final double trapPosition = 2.27;
  public static final double ampPosition = 0.6; // 0.6
  public static final double ShootDeflect = 0.18; // 0.24

  public static final double positionError = 0.05;
}
