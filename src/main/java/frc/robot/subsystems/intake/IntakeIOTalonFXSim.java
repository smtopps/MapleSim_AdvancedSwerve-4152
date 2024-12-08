// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/** Add your docs here. */
public class IntakeIOTalonFXSim implements IntakeIO {
  private final TalonFX rotationMotor = new TalonFX(IntakeConstants.rotationMotorID);
  private TalonFXSimState rotationMotorSim = rotationMotor.getSimState();
  private final SparkFlex rollerMotor =
      new SparkFlex(IntakeConstants.rollerMotorID, MotorType.kBrushless);
  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);
  private SingleJointedArmSim rotationSim;

  /**
   * @param driveTrain the swerve drivetrain simulation to which this intake is attached
   * @param passNoteToFlyWheelsCall called when the note in the intake is pushed to the flywheels,
   *     allowing the flywheels to simulate the projected note
   */
  public IntakeIOTalonFXSim(
      AbstractDriveTrainSimulation driveTrain, Runnable passNoteToFlyWheelsCall) {
    rotationSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            IntakeConstants.rotationGearRatio,
            0.05457986,
            0.168484569,
            Units.rotationsToRadians(-0.15),
            // Units.rotationsToRadians(-0.5),
            Units.rotationsToRadians(0.5),
            true,
            IntakeConstants.stowedPosition);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    rotationMotorSim.setSupplyVoltage(12.0);
    double motorVoltage = rotationMotorSim.getMotorVoltage();
    rotationSim.setInputVoltage(motorVoltage);
    rotationSim.update(0.02);
    rotationMotorSim.setRawRotorPosition(
        IntakeConstants.rotationGearRatio * Units.radiansToRotations(rotationSim.getAngleRads()));
    rotationMotorSim.setRotorVelocity(
        IntakeConstants.rotationGearRatio
            * Units.radiansToRotations(rotationSim.getVelocityRadPerSec()));

    // detect the note with the beam breaker and store it in "inputs"
    // if the note is present, the beam breaker is blocked and will return "false"
    inputs.noteDetected = true;
    inputs.intakePose =
        new Pose3d(
            0.32004,
            0,
            0.280616,
            new Rotation3d(
                0,
                -Units.rotationsToRadians(rotationMotor.getPosition().getValueAsDouble() - 0.448),
                0));
    inputs.intakeRotation = rotationMotor.getPosition().getValueAsDouble();
  }

  public void configureRotationMotor() {
    rotationMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rotationMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration talonfxConfigs = new TalonFXConfiguration();
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
    rollerMotor.configure(
        new SparkFlexConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkFlexConfig sparkFlexConfigs = new SparkFlexConfig();
    sparkFlexConfigs
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.rollerMotorCurrentLimit);
    rollerMotor.configure(
        sparkFlexConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.set(IntakeConstants.stallSpeed);
  }

  public void setIntakeRollerCurrentLimit(int current) {
    SparkFlexConfig sparkFlexConfigs = new SparkFlexConfig();
    sparkFlexConfigs.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(current);
    rollerMotor.configure(
        sparkFlexConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
