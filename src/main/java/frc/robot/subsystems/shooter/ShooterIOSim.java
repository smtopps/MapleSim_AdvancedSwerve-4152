// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim leftSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.004, 0.8),
          DCMotor.getKrakenX60(1));
  private final FlywheelSim rightSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.004, 0.8),
          DCMotor.getKrakenX60(1));
  private PIDController pid = new PIDController(10.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private double leftSetpoint = 0;
  private double rightSetpoint = 0;

  @Override
  public void updateInputs(ShooterInputs inputs) {
    if (closedLoop) {
      pid.setSetpoint(leftSetpoint);
      leftSim.setInputVoltage(pid.calculate(leftSim.getAngularVelocityRPM() / 60.0));
      pid.setSetpoint(rightSetpoint);
      rightSim.setInputVoltage(pid.calculate(rightSim.getAngularVelocityRPM() / 60.0));
    } else {
      leftSim.setInputVoltage(0.0);
      rightSim.setInputVoltage(0.0);
    }

    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.leftVelocityRPS = leftSim.getAngularVelocityRPM() / 60;
    inputs.rightVelocityRPS = rightSim.getAngularVelocityRPM() / 60;
  }

  @Override
  public void configureShooterMotors() {}

  @Override
  public void setShooterSpeeds(double RPS, double spinFactor) {
    closedLoop = true;
    leftSetpoint = RPS * (1 - spinFactor);
    rightSetpoint = RPS * (1 + spinFactor);
  }

  @Override
  public void stopShooter() {
    closedLoop = false;
    leftSetpoint = 0;
    rightSetpoint = 0;
  }

  /**
   * when the intake passes the note to the flywheels, this method is called to simulate launching a
   * note from the shooter
   */
  public void shootNoteWithCurrentRPM(
      Pose2d robotSimulationWorldPose, ChassisSpeeds chassisSpeedsFieldRelative) {
    double avgRPM = (leftSim.getAngularVelocityRPM() + rightSim.getAngularVelocityRPM()) / 2.0;
    double wheelSlip = 1.33 + (-0.0227 * avgRPM / 60) + (0.000135 * (Math.pow(avgRPM / 60, 2.0)));
    double linearSpeed = Math.PI * Units.inchesToMeters(4.0) * avgRPM / 60 * wheelSlip;
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new NoteOnFly(
                    robotSimulationWorldPose
                        .getTranslation(), // specify the position of the chassis
                    new Translation2d(
                        0.26, 0.0), // the shooter is installed at this position on the robot (in
                    // reference to the robot chassis center)
                    chassisSpeedsFieldRelative, // specify the field-relative speed of the chassis
                    // to add it to the initial velocity of the projectile
                    robotSimulationWorldPose
                        .getRotation()
                        .plus(Rotation2d.k180deg), // the shooter facing is the robot's facing
                    0.6, // initial height of the flying note
                    linearSpeed,
                    Math.toRadians(38) // the note is launched at fixed angle of 38 degrees.
                    )
                .asSpeakerShotNote(() -> System.out.println("hit speaker!!!"))
                .enableBecomeNoteOnFieldAfterTouchGround()
                .withProjectileTrajectoryDisplayCallBack(
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/NoteProjectileSuccessful", pose3ds.toArray(Pose3d[]::new)),
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/NoteProjectileUnsuccessful",
                            pose3ds.toArray(Pose3d[]::new))));
  }

  public void ampNoteWithCurrentRPM(
    Pose2d robotSimulationWorldPose, ChassisSpeeds chassisSpeedsFieldRelative) {
      double linearSpeed = Math.PI * Units.inchesToMeters(4.0) * ShooterConstants.ampHandoffRPS;
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new NoteOnFly(
                      robotSimulationWorldPose
                          .getTranslation(), // specify the position of the chassis
                      new Translation2d(
                          0.26, 0.0), // the shooter is installed at this position on the robot (in
                      // reference to the robot chassis center)
                      chassisSpeedsFieldRelative, // specify the field-relative speed of the chassis
                      // to add it to the initial velocity of the projectile
                      robotSimulationWorldPose
                          .getRotation()
                          .plus(Rotation2d.k180deg), // the shooter facing is the robot's facing
                      0.6, // initial height of the flying note
                      linearSpeed,
                      Math.toRadians(38) // the note is launched at fixed angle of 38 degrees.
                      )
                  .asAmpShotNote(() -> System.out.println("hit amp!!!"))
                  .enableBecomeNoteOnFieldAfterTouchGround()
                  .withProjectileTrajectoryDisplayCallBack(
                      (pose3ds) ->
                          Logger.recordOutput(
                              "Flywheel/NoteProjectileSuccessful", pose3ds.toArray(Pose3d[]::new)),
                      (pose3ds) ->
                          Logger.recordOutput(
                              "Flywheel/NoteProjectileUnsuccessful",
                              pose3ds.toArray(Pose3d[]::new))));
  };
}
