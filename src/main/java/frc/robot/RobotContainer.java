// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands.AutoAlignNotes;
import frc.robot.commands.AutoCommands.AutoIntakeStart;
import frc.robot.commands.AutoCommands.AutoIntakeStop;
import frc.robot.commands.AutoCommands.AutoManualShoot;
import frc.robot.commands.AutoCommands.AutoShootDeflect;
import frc.robot.commands.AutoCommands.AutoShootOnTheFly;
import frc.robot.commands.AutoCommands.AutoShootPose;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands.IntakeAlignAndDriveToNote;
import frc.robot.commands.IntakeCommands.IntakeAmp;
import frc.robot.commands.IntakeCommands.IntakeGround;
import frc.robot.commands.ShootCommands.ManualShoot;
import frc.robot.commands.ShootCommands.RevShooter;
import frc.robot.commands.ShootCommands.ShootDeflect;
import frc.robot.commands.ShootCommands.ShootPose;
import frc.robot.commands.ShootCommands.ShootToZone;
import frc.robot.commands.TrapAmpCommands.AlignAmpSequence;
import frc.robot.commands.TrapAmpCommands.AmpSequence;
import frc.robot.commands.TrapAmpCommands.ClimberHold;
import frc.robot.commands.TrapAmpCommands.ClimberManual;
import frc.robot.commands.TrapAmpCommands.TrapSequence;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrap;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrapIO;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrapIOReal;
import frc.robot.subsystems.ElevatorTrap.ElevatorTrapIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation.DRIVE_WHEEL_TYPE;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Simulations are store in the robot container
  private final SwerveDriveSimulation swerveDriveSimulation;

  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final ElevatorTrap elevatorTrap;
  private final Climber climber;
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        /* Real robot, instantiate hardware IO implementations */

        /* Disable Simulations */
        this.swerveDriveSimulation = null;

        /* Subsystems */
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        shooter = new Shooter(new ShooterIOTalonFX());
        this.intake = new Intake(new IntakeIOTalonFX());
        elevatorTrap = new ElevatorTrap(new ElevatorTrapIOReal());
        climber = new Climber(new ClimberIOReal());
        vision = new Vision(new VisionIOLimelight(), drive);
        break;

      case SIM:
        /* Sim robot, instantiate physics sim IO implementations */

        /* create simulation for pigeon2 IMU (different IMUs have different measurement errors) */
        final GyroSimulation gyroSimulation = GyroSimulation.createPigeon2();
        /* create a swerve drive simulation */
        this.swerveDriveSimulation =
            new SwerveDriveSimulation(
                62,
                Units.inchesToMeters(20.75),
                Units.inchesToMeters(22.75),
                Units.inchesToMeters(32.875),
                Units.inchesToMeters(34.875),
                SwerveModuleSimulation.getMark4i( // creates a mark4i module
                    DCMotor.getKrakenX60(1), // drive motor is a Kraken x60
                    DCMotor.getFalcon500(1), // steer motor is a Falcon 500
                    50, // current limit: 50 Amps
                    DRIVE_WHEEL_TYPE.RUBBER, // rubber wheels
                    2 // l2 gear ratio
                    ),
                gyroSimulation,
                new Pose2d( // initial starting pose on field, set it to where-ever you want
                    3, 3, new Rotation2d()));
        SimulatedArena.getInstance()
            .addDriveTrainSimulation(swerveDriveSimulation); // register the drive train simulation

        // reset the field for auto (placing game-pieces in positions)
        SimulatedArena.getInstance().resetFieldForAuto();

        drive =
            new Drive(
                new GyroIOSim(
                    gyroSimulation), // GyroIOSim is a wrapper around gyro simulation, that reads
                // the simulation result
                /* ModuleIOSim are edited such that they also wraps around module simulations */
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]));

        /* other subsystems are created with hardware simulation IOs */
        final ShooterIOSim shooterIOSim = new ShooterIOSim();
        shooter = new Shooter(shooterIOSim);
        this.elevatorTrap = new ElevatorTrap(new ElevatorTrapIOSim());
        this.climber = new Climber(new ClimberIOSim());

        this.intake =
            new Intake(
                new IntakeIOSim(
                    swerveDriveSimulation, // attach the intake simulation to swerve drive
                    () ->
                        shooterIOSim.shootNoteWithCurrentRPM(
                            swerveDriveSimulation.getSimulatedDriveTrainPose(),
                            swerveDriveSimulation
                                .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            elevatorTrap.getElevatorPosition())));
        this.vision = new Vision(new VisionIOPhotonSim(swerveDriveSimulation), drive);
        // simulation
        break;

      default:
        /* Replayed robot, disable IO implementations */

        /* physics simulations are also not needed */
        this.swerveDriveSimulation = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});

        this.intake = new Intake((inputs) -> {});
        elevatorTrap = new ElevatorTrap(new ElevatorTrapIO() {});
        climber = new Climber(new ClimberIO() {});
        vision = new Vision(new VisionIO() {}, drive);
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand("startIntake", new AutoIntakeStart(intake));
    NamedCommands.registerCommand("stopIntake", new AutoIntakeStop(intake));
    NamedCommands.registerCommand("autoShoot", new AutoShootPose(drive, shooter, intake));
    NamedCommands.registerCommand("autoAlign", new AutoAlignNotes(drive, intake));
    NamedCommands.registerCommand("deflect", new AutoShootDeflect(shooter, intake, elevatorTrap));
    NamedCommands.registerCommand("manuelShoot", new AutoManualShoot(shooter, intake));
    NamedCommands.registerCommand("moveShoot", new AutoShootOnTheFly(drive, shooter, intake));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  /**
   * Use this method to define your driver button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    shooter.setDefaultCommand(
        new RevShooter(drive, shooter).onlyWhile(() -> DriverStation.isTeleop()));
    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            this.swerveDriveSimulation == null
                                ? new Pose2d(drive.getPose().getTranslation(), new Rotation2d())
                                : swerveDriveSimulation.getSimulatedDriveTrainPose()),
                    drive)
                .ignoringDisable(true));

    driverController.rightBumper().whileTrue(new IntakeGround(intake));
    driverController
        .rightTrigger(0.05)
        .whileTrue(
            new IntakeAlignAndDriveToNote(
                intake,
                drive,
                () -> driverController.getRightTriggerAxis(),
                () -> -driverController.getLeftY() * DriveConstants.MAX_LINEAR_SPEED,
                () -> -driverController.getLeftX() * DriveConstants.MAX_LINEAR_SPEED,
                () -> -driverController.getRightX() * DriveConstants.MAX_ANGULAR_SPEED));
    driverController
        .leftBumper()
        .whileTrue(new ManualShoot(shooter, intake, ShooterConstants.shootingRPS));
    driverController
        .leftTrigger()
        .whileTrue(new ShootPose(drive, shooter, intake).ignoringDisable(true));
    driverController.y().whileTrue(new IntakeAmp(intake));
    driverController.b().onTrue(new AmpSequence(elevatorTrap, intake, shooter));
    driverController
        .rightStick()
        .whileTrue(new AlignAmpSequence(intake, drive, elevatorTrap, shooter));
    driverController.a().whileTrue(new ShootDeflect(shooter, intake, elevatorTrap));
    driverController
        .leftStick()
        .whileTrue(
            new ShootToZone(
                drive,
                shooter,
                intake,
                () -> -driverController.getLeftY() * DriveConstants.MAX_LINEAR_SPEED,
                () -> -driverController.getLeftX() * DriveConstants.MAX_LINEAR_SPEED));
  }

  /**
   * Use this method to define your operator button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureOperatorButtonBindings() {
    operatorController
        .a()
        .whileTrue(new ClimberManual(climber, () -> -operatorController.getLeftY()))
        .onFalse(new ClimberHold(climber));
    // operatorController.y().whileTrue(new ElevatorManual(elevatorTrap, ()->
    // operatorController.getLeftY())).onFalse(new ElevatorHold(elevatorTrap));
    // operatorController.x().whileTrue(new TrapManual(trap, ()-> operatorController.getLeftY()));
    // operatorController.back().onTrue(new InstantCommand(()-> climber.resetEncoder(true),
    // climber));
    operatorController.rightBumper().onTrue(new TrapSequence(elevatorTrap, intake, shooter));
    // operatorController.rightTrigger().whileTrue(new IntakeTryHarder(intake));
    // operatorController.povCenter().whileFalse(new IntakeAmp(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.get();
    return autoCommand;
  }

  public void updateSimulationField() {
    if (swerveDriveSimulation != null) {
      SimulatedArena.getInstance().simulationPeriodic();

      Logger.recordOutput(
          "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());

      final List<Pose3d> notes = SimulatedArena.getInstance().getGamePiecesByType("Note");
      if (notes != null) Logger.recordOutput("FieldSimulation/Notes", notes.toArray(Pose3d[]::new));
    }

    intake.visualizeNoteInIntake(
        swerveDriveSimulation == null
            ? drive.getPose()
            : swerveDriveSimulation.getSimulatedDriveTrainPose());
  }
}
