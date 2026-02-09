// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // CAN IDs

  // Aliases: Magazine motors
  public static final int MAIN_ROLLER_MOTOR_CAN_ID = 25; // Right roller motor.
  public static final int FOLLOWER_ROLLER_MOTOR_CAN_ID = 22; // Left roller motor.

  // When robot front is facing away, this is the right feeder motor.
  // Aliases: Vectoring motors
  public static final int LEFT_FEEDER_MOTOR_ID = 23;
  public static final int RIGHT_FEEDER_MOTOR_ID = 24;

  public static final int INTAKE_MOTOR_CAN_ID = 26;
  public static final int EXTENDER_MOTOR_CAN_ID = 27;
}
