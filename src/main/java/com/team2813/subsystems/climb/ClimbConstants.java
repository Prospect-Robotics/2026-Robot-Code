package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants {
  public static final double INNER_MOTOR_TO_CLIMB_GEARING = 9;
  public static final double OUTER_MOTOR_TO_CLIMB_GEARING = 9;

  public static final TalonFXConfiguration INNER_MOTOR_TO_CLIMB_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(INNER_MOTOR_TO_CLIMB_GEARING))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  public static final TalonFXConfiguration OUTER_MOTOR_TO_CLIMB_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(OUTER_MOTOR_TO_CLIMB_GEARING))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  public static final Mass INNER_CLIMB_CARRIAGE_WEIGHT = Pounds.of(4);
  public static final Mass OUTER_CLIMB_CARRIAGE_WEIGHT = Pounds.of(4);

  // NOTE: These may be the same value, because it seems both motors have the same gearbox.
  public static final double INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0;
  public static final double OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0;

  public static final Distance INNER_CLIMB_SPOOL_RADIUS = Millimeters.of(13.75);
  public static final Distance OUTER_CLIMB_SPOOL_RADIUS = Millimeters.of(13.75);

  public static final Distance INNER_CLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance INNER_CLIMB_MAX_HEIGHT = Inches.of(28.0);

  public static final Distance OUTER_CLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance OUTER_CLIMB_MAX_HEIGHT = Inches.of(28.0);
}
