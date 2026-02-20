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
  public static final double INNERCLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0;
  public static final double OUTERCLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0;
  public static final Distance INNERCLIMB_SPOOL_RADIUS = Inches.of(0);
  public static final Distance OUTERCLIMB_SPOOL_RADIUS = Inches.of(0);

  public static final Distance INNERCLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance INNERCLIMB_MAX_HEIGHT = Inches.of(28.0);
  public static final Distance OUTERCLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance OUTERCLIMB_MAX_HEIGHT = Inches.of(28.0);
}
