// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PIDToPose;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAmpSequence extends SequentialCommandGroup {
  Pose2d blueAmpPose = new Pose2d(1.83, 7.62, Rotation2d.fromDegrees(-90.0));
  Pose2d redAmpPose = new Pose2d(14.72, 7.62, Rotation2d.fromDegrees(-90.0));
  /** Creates a new AutoAmp. */
  public AlignAmpSequence(
      Intake intake, Drive drivetrain, ElevatorTrap elevatorTrap, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
            new PIDToPose(drivetrain, blueAmpPose),
            new PIDToPose(drivetrain, redAmpPose),
            () -> DriverStation.getAlliance().get().equals(Alliance.Blue)),
        new ParallelRaceGroup(
            new InstantCommand(() -> drivetrain.runVelocity(new ChassisSpeeds(-0.6, 0, 0))),
            new WaitCommand(0.3)),
        new AmpSequence(elevatorTrap, intake, shooter));
  }
}
