package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants {
  // Gotten from design and fab.
  public static final double LEFTMOTOR_TO_CLIMB_GEARING = 9;
  public static final double RIGHTMOTOR_TO_CLIMB_GEARING = 9;
  public static final TalonFXConfiguration LEFTMOTOR_TO_CLIMB_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(LEFTMOTOR_TO_CLIMB_GEARING));
  public static final TalonFXConfiguration RIGHTMOTOR_TO_CLIMB_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(RIGHTMOTOR_TO_CLIMB_GEARING));

  public static final Mass INNERCLIMB_CARRIAGE_WEIGHT = Pounds.of(0);
  public static final Mass OUTERCLIMB_CARRIAGE_WEIGHT = Pounds.of(0);
  // Correct calculation for Inches of travel per 1 Motor Rotation:
  // (2 * pi * Spool Radius) / Gearing
  // NOTE to daniel: Will need to recalculate this with the new CLIMB_SPOOL_RADIUS
  public static final double INNERCLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0; // 0.272049
  public static final double OUTERCLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0; // 0.272049
  // Guess what, I asked design again.
  public static final Distance INNERCLIMB_SPOOL_RADIUS = Inches.of(0);
  public static final Distance OUTERCLIMB_SPOOL_RADIUS = Inches.of(0);

  public static final Distance INNERCLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance INNERCLIMB_MAX_HEIGHT =
      Inches.of(28.0); // This is the max height of the first stage, which we simulate.

  public static final Distance OUTERCLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance OUTERCLIMB_MAX_HEIGHT =
      Inches.of(28.0); // This is the max height of the first stage, which we simulate.

  public static final double LEFTCLIMB_kG = 0.0; // 0.29
  public static final double LEFTCLIMB_kS = 0.0; // 0.11
  public static final double LEFTCLIMB_kV = 0.0; // 0.1
  public static final double LEFTCLIMB_kA = 0.0;
  public static final double LEFTCLIMB_kP = 0.1;
  public static final double LEFTCLIMB_kI = 0.0;
  public static final double LEFTCLIMB_kD = 0.0;
}
