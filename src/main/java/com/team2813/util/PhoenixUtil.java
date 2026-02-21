// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Get timestamps (in seconds) for the individual odometry simulation ticks.
   *
   * <p>Per https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/, the
   * MapleSim maintains high-frequency ticks odometry. For AdvantageKit, the robot simulation must
   * stay at 50Hz (i.e., 0.02s per iteration), but it can have a multiple of odometry ticks per
   * iteration, where by default MapleSim uses 5 ticks per iteration.
   *
   * <p>This function returns the timestamps of all odometry simulation ticks _since_ the start of
   * the last robot period (i.e., {@code Timer.getFPGATimestamp() - 0.02}).
   *
   * <p>Function borrowed from
   * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/util/PhoenixUtil.java.
   *
   * @return Timestamps (in seconds) of last robot period odometry simluation ticks.
   */
  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }
}
