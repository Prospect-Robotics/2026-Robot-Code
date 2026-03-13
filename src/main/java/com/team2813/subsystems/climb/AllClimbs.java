package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.team2813.Constants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class AllClimbs {
  public static AllClimbConstants outerClimb() {
    return new AllClimbConstants(
        "Outer",
        Constants.OUTER_CLIMB_MOTOR_ID,
        ClimbConstants.OUTER_MOTOR_TO_CLIMB_GEARING,
        ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG,
        ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION,
        ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS,
        ClimbConstants.OUTER_CLIMB_MIN_HEIGHT,
        ClimbConstants.OUTER_CLIMB_MID_HEIGHT,
        ClimbConstants.OUTER_CLIMB_MAX_HEIGHT);
  }

  public static AllClimbConstants innerClimb() {
    return new AllClimbConstants(
        "Inner",
        Constants.INNER_CLIMB_MOTOR_ID,
        ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING,
        ClimbConstants.INNER_MOTOR_TO_CLIMB_CONFIG,
        ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION,
        ClimbConstants.INNER_CLIMB_SPOOL_RADIUS,
        ClimbConstants.INNER_CLIMB_MIN_HEIGHT,
        ClimbConstants.INNER_CLIMB_MID_HEIGHT,
        ClimbConstants.INNER_CLIMB_MAX_HEIGHT);
  }

  public enum InnerClimbHeight {
    // elliot said add 3 inches since its not a normal elevator beacuse a rope is spolling it,
    // except for down
    // Origional values UP(Inches.of(9.75)), MIDDLE(Inches.of(4.875)),
    UP(Inches.of(12.75)),
    // TODO figure post auto position
    POSTAUTO(Inches.of(4)),
    MIDDLE(Inches.of(7.875)),
    DOWN(Inches.of(0.0));

    public final Distance position;

    InnerClimbHeight(Distance position) {
      this.position = position;
    }

    public Distance getInnerPosition() {
      return position;
    }
  }

  public enum UpperClimbHeight {
    // elliot said add 3 inches since its not a normal elevator beacuse a rope is spolling it,
    // except for down
    // Origional values UP(Inches.of(11)), MIDDLE(Inches.of(5.5)),
    UP(Inches.of(14)),
    MIDDLE(Inches.of(8.5)),
    DOWN(Inches.of(0.0));

    public final Distance position;

    UpperClimbHeight(Distance position) {
      this.position = position;
    }

    public Distance getPosition() {
      return position;
    }
  }
}
