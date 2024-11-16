// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSimulation;
  private final AbstractDriveTrainSimulation driveTrain;
  private final Runnable passNoteToFlyWheelsCall;
  private double rollerSpeed,
      // This is an indefinite integral of the intake motor voltage since the note has been in the
      // intake.
      // This approximates the position of the note in the intake.
      intakeVoltageIntegralSinceNoteTaken = 0.0;
  private double rotationPosition;
  private SingleJointedArmSim rotationSim;
  private PIDController rotationController;

  /**
   * @param driveTrain the swerve drivetrain simulation to which this intake is attached
   * @param passNoteToFlyWheelsCall called when the note in the intake is pushed to the flywheels,
   *     allowing the flywheels to simulate the projected note
   */
  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain, Runnable passNoteToFlyWheelsCall) {
    rotationSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            IntakeConstants.rotationGearRatio,
            0.1401,
            Units.inchesToMeters(3.5307879437063),
            Units.rotationsToRadians(-0.132),
            Units.rotationsToRadians(0.498),
            false,
            Units.rotationsToRadians(IntakeConstants.stowedPosition));
    // 0);
    rotationController = new PIDController(10.0, 0.0, 0.0);
    rotationController.setTolerance(0.1); // 0.01

    this.intakeSimulation =
        new IntakeSimulation( // create intake simulation with no extension
            "Note", // the intake grabs game pieces of this type
            driveTrain, // specify the drivetrain to which the intake is attached to
            0.3, // the width of the intake 0.4
            IntakeSimulation.IntakeSide.FRONT, // the intake is attached the back of the drivetrain
            1 // the intake can only hold 1 game piece at a time
            );
    intakeSimulation.register();

    this.driveTrain = driveTrain;
    this.passNoteToFlyWheelsCall = passNoteToFlyWheelsCall;
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // gamePiecesInIntakeCount shows the amount of game pieces in the intake, we store this in the
    // inputs
    inputs.noteDetected = intakeSimulation.getGamePiecesAmount() != 0;

    calculateRotaion();
    inputs.intakePose =
        new Pose3d(
            0.32004,
            0,
            0.280616,
            new Rotation3d(
                0,
                -Units.rotationsToRadians(
                    Units.radiansToRotations(rotationSim.getAngleRads()) - 0.448),
                0));

    inputs.intakeRotation = Units.radiansToRotations(rotationSim.getAngleRads());

    if (rotationController.atSetpoint() && rollerSpeed > 0.2) intakeSimulation.startIntake();
    else intakeSimulation.stopIntake();

    // if the there is note, we do an integral to the voltage to approximate the position of the
    // note in the intake
    if (inputs.noteDetected) intakeVoltageIntegralSinceNoteTaken += 0.16 * rollerSpeed;
    // if the note is gone, we clear the integral
    else intakeVoltageIntegralSinceNoteTaken = 0.0;
    if (inputs.noteDetected && rollerSpeed >= 0) intakeVoltageIntegralSinceNoteTaken = 0;

    // if the integral is negative, we get rid of the note
    if (intakeVoltageIntegralSinceNoteTaken < 0
        && Units.radiansToRotations(rotationSim.getAngleRads()) < 0.3
        && intakeSimulation.obtainGamePieceFromIntake())
      // splits the note out by adding it on field
      SimulatedArena.getInstance()
          .addGamePiece(
              new CrescendoNoteOnField(
                  driveTrain
                      .getSimulatedDriveTrainPose()
                      .getTranslation()
                      .plus(
                          new Translation2d(1.0, 0.0) // -0.4, 0
                              .rotateBy(driveTrain.getSimulatedDriveTrainPose().getRotation()))));
    else if (intakeVoltageIntegralSinceNoteTaken < 0
        && Units.radiansToRotations(rotationSim.getAngleRads()) >= 0.3
        && intakeSimulation.obtainGamePieceFromIntake()) passNoteToFlyWheelsCall.run();
  }

  @Override
  public void configureRotationMotor() {
    setIntakePosition(IntakeConstants.stowedPosition);
  }

  @Override
  public void configureRollerMotor() {}

  @Override
  public void setIntakeRollerCurrentLimit(int current) {}

  @Override
  public void setIntakePosition(double position) {
    rotationPosition = position;
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerSpeed = speed;
  }

  @Override
  public void stopRotation() {}

  @Override
  public void stopRoller() {
    rollerSpeed = 0.0;
  }

  private void calculateRotaion() {
    rotationController.setSetpoint(Units.rotationsToRadians(rotationPosition));
    rotationSim.setInputVoltage(rotationController.calculate(rotationSim.getAngleRads()));
    rotationSim.update(0.02);
  }
}
