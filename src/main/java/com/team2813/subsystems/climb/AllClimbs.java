package com.team2813.subsystems.climb;

import com.team2813.Constants;

public class AllClimbs {
  public static AllClimbConstants outerClimb() {
    AllClimbConstants climbConstants =
        new AllClimbConstants(
            "Outer",
            Constants.OUTER_CLIMB_MOTOR_ID,
            ClimbConstants.OUTER_MOTOR_TO_CLIMB_GEARING,
            ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG,
            ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION,
            ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS,
            ClimbConstants.OUTER_CLIMB_MIN_HEIGHT,
            ClimbConstants.OUTER_CLIMB_MID_HEIGHT,
            ClimbConstants.OUTER_CLIMB_MAX_HEIGHT);

    return climbConstants;
  }

  public static AllClimbConstants innerClimb() {
    AllClimbConstants climbConstants =
        new AllClimbConstants(
            "Inner",
            Constants.INNER_CLIMB_MOTOR_ID,
            ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING,
            ClimbConstants.INNER_MOTOR_TO_CLIMB_CONFIG,
            ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION,
            ClimbConstants.INNER_CLIMB_SPOOL_RADIUS,
            ClimbConstants.INNER_CLIMB_MIN_HEIGHT,
            ClimbConstants.INNER_CLIMB_MID_HEIGHT,
            ClimbConstants.INNER_CLIMB_MAX_HEIGHT);

    return climbConstants;
  }
}
