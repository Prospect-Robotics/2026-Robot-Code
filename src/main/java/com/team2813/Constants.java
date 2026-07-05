// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO.
 */
public final class Constants {
  public static double SIM_TIME_PERIOD =
      0.02; // Update physics simulations every 20ms (like the actual bot).

  // CAN IDs - All directions are from when the robot is viewed from behind, unless otherwise
  // stated.
  // Roller Motors. Aliases: Feeder motors, Magazine motors
  // These force balls from the hopper into the shooter.
  public static final int FEEDER_MOTOR_CAN_ID = 25; // Feeder motor.

  // Indexer Motors. Aliases: Vectoring motors
  public static final int INDEXER_MOTOR_ID = 24;

  // Motors that run the robot intake.
  public static final int LEFT_INTAKE_MOTOR_CAN_ID = 26;
  public static final int RIGHT_INTAKE_MOTOR_CAN_ID = 28;
  // Motor controls the extension of the front of the hopper plate.
  public static final int EXTENDER_MOTOR_CAN_ID = 27;

  // NOTE: The below motors are with placeholder CANIDs and are subject to change.
  // TODO: Discuss with electrical for permanent IDs.
  // Shooter Motors. Aliases: Flywheel motors.
  public static final int UPPER_RIGHT_SHOOTER_MOTOR_ID = 19; // Upper right shooter motor.
  public static final int LOWER_RIGHT_SHOOTER_MOTOR_ID = 30; // Lower right shooter motor.

  public static final int UPPER_LEFT_SHOOTER_MOTOR_ID = 20; // Upper left shooter motor.
  public static final int LOWER_LEFT_SHOOTER_MOTOR_ID = 31; // Lower left shooter motor.

  // Kicker Motor
  public static final int KICKER_MOTOR_ID = 21;

  public static final int HOOD_MOTOR_ID = 23;

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
