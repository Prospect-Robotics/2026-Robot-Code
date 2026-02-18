package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants {
  // Gotten from design and fab.
  public static final double LEFTMOTOR_TO_CLIMB_GEARING = 9;

  // NOTE: Takes arm weight into account.
  public static final Mass LEFTCLIMB_CARRIAGE_WEIGHT = Pounds.of(0);

  // Correct calculation for Inches of travel per 1 Motor Rotation:
  // (2 * pi * Spool Radius) / Gearing
  // NOTE to daniel: Will need to recalculate this with the new CLIMB_SPOOL_RADIUS
  public static final double LEFTCLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0.681933; // 0.272049

  // Guess what, I asked design again.
  public static final Distance LEFTCLIMB_SPOOL_RADIUS = Inches.of(0.8755);

  public static final Distance LEFTCLIMB_MIN_HEIGHT = Inches.of(0.0);
  public static final Distance LEFTCLIMB_MAX_HEIGHT =
      Inches.of(28.0); // This is the max height of the first stage, which we simulate.

  public static final double LEFTCLIMB_kG = 0.0; // 0.29
  public static final double LEFTCLIMB_kS = 0.0; // 0.11
  public static final double LEFTCLIMB_kV = 0.0; // 0.1
  public static final double LEFTCLIMB_kA = 0.0;
  public static final double LEFTCLIMB_kP = 0.1;
  public static final double LEFTCLIMB_kI = 0.0;
  public static final double LEFTCLIMB_kD = 0.0;
}
