// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static double SIM_TIME_PERIOD =
      0.02; // Update physics simulations every 20ms (like the actual bot).

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Transform3d FRONT_CAMERA_POSITION =
      new Transform3d(
          new Translation3d(Meters.of(-0.054564), Meters.of(0), Meters.of(0.501754)),
          new Rotation3d(Degree.of(0), Degree.of(-29.5), Degree.of(0)));

  // CAN IDs - All directions are from when the robot is viewed from behind, unless otherwise
  // stated.
  // Roller Motors. Aliases: Magazine motors
  public static final int MAIN_ROLLER_MOTOR_CAN_ID = 25; // Top roller motor.
  public static final int FOLLOWER_ROLLER_MOTOR_CAN_ID = 15; // Bottom roller motor.

  // Feeder Motors. Aliases: Vectoring motors
  public static final int FEEDER_MOTOR_ID = 24;

  // Motor runs the robot intake.
  public static final int INTAKE_MOTOR_CAN_ID = 26;
  // Motor controls the extension of the front of the hopper plate.
  public static final int EXTENDER_MOTOR_CAN_ID = 27;

  // NOTE: The below motors are with placeholder CANIDs and are subject to change.
  // TODO: Discuss with electrical for permanent IDs.
  // Shooter Motors. Aliases: Flywheel motors.
  public static final int MAIN_SHOOTER_MOTOR_ID = 19; // Right shooter motor.
  public static final int FOLLOWER_SHOOTER_MOTOR_ID = 20; // Left shooter motor.

  // Kicker Motor
  public static final int KICKER_MOTOR_ID = 21;

  /**
   * Returns true if the robot is on the red alliance.
   *
   * <p>Defaults to blue if no alliance is present.
   *
   * @return - true if the robot is on the red alliance, false otherwise.
   */
  public static boolean onRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }
}
