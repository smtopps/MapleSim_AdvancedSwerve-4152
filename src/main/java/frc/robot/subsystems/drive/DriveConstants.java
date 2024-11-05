// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
  public static final double MAX_LINEAR_SPEED = 4.35;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(22.75);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  public static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          62,
          6.0,
          new ModuleConfig(
              Units.inchesToMeters(2), MAX_LINEAR_SPEED, 1.2, DCMotor.getKrakenX60(1), 50, 1),
          TRACK_WIDTH_Y,
          TRACK_WIDTH_X);
}
