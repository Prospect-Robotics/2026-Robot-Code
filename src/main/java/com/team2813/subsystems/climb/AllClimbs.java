package com.team2813.subsystems.climb;

public class AllClimbs {
private static AllClimbConstants outerClimb(){
    AllClimbConstants climbConstants = 
        new AllClimbConstants("Outer", ClimbConstants.OUTER_MOTOR_TO_CLIMB_GEARING, ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG, ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION, 
        ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS, ClimbConstants.OUTER_CLIMB_MIN_HEIGHT, ClimbConstants.OUTER_CLIMB_MIN_HEIGHT); 

    return climbConstants; 
    }
}
