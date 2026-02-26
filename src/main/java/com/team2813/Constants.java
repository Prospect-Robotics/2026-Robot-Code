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
 * on a roboRIO.
 */
public final class Constants {
  /**
   * Gets the {@link Mode} that the robot should use in simulation. This method will never return
   * {@link Mode#REAL}, as that is never appropriate for robot simulation. If the gradle {@code
   * replayWatch} task is run, this will return {@link Mode#REPLAY} automatically.
   *
   * @return The {@link Mode} that the robot should use if it is being simulated.
   */
  static Mode getSimMode() {
    // The environment variable "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE" is set to "true" when running
    // `replayWatch`. This will then only return `Mode.REPLAY` when we are in replay mode. Note that
    // this will set `Mode.REPLAY` if the user sets this environment variable, but that is probably
    // not going to happen due to the long, specific name, and if it does, that is their problem :3.
    if (Boolean.parseBoolean(System.getenv("FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE"))) {
      return Mode.REPLAY;
    } else {
      return Mode.SIM;
    }
  }

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : getSimMode();

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

  // Climb motors
  // TODO(Stefan): ID these motors!
  public static final int LEFTCLIMB_MOTOR_ID = 30;
  public static final int RIGHTCLIMB_MOTOR_ID = 31;
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
