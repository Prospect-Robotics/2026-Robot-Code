package com.team2813.subsystems.climb;

public class AllClimbs {
    public static AllClimbConstants outerClimb() {
        AllClimbConstants climbConstants =
                new AllClimbConstants(
                        "Outer",
                        ClimbConstants.OUTER_MOTOR_TO_CLIMB_GEARING,
                        ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG,
                        ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION,
                        ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS,
                        ClimbConstants.OUTER_CLIMB_MIN_HEIGHT,
                        ClimbConstants.OUTER_CLIMB_MAX_HEIGHT);

        return climbConstants;
    }

    public static AllClimbConstants innerClimb() {
        AllClimbConstants climbConstants =
                new AllClimbConstants(
                        "Inner",
                        ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING,
                        ClimbConstants.INNER_MOTOR_TO_CLIMB_CONFIG,
                        ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION,
                        ClimbConstants.INNER_CLIMB_SPOOL_RADIUS,
                        ClimbConstants.INNER_CLIMB_MIN_HEIGHT,
                        ClimbConstants.INNER_CLIMB_MAX_HEIGHT

                );

        return climbConstants;
    }


}
