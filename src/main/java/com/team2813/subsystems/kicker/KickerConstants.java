package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.MomentOfInertia;

class KickerConstants {
  static final MomentOfInertia KICKER_SIM_MOI = KilogramSquareMeters.of(0.0000535531);

  static final MomentOfInertia UPPER_KICKER_MOI = KilogramSquareMeters.of(0.0094843);
  static final MomentOfInertia LOWER_KICKER_MOI = KilogramSquareMeters.of(0.003038495);
  static final String UPPER_SHOOT_PREFERENCE_NT = "Kicker/UPPER_SHOOT_VOLTAGE";
  static final String LOWER_SHOOT_PREFERENCE_NT = "Kicker/LOWER_SHOOT_VOLTAGE";
  static final String UPPER_RESIST_FUEL_PREFERENCE_NT = "Kicker/UPPER_RESIST_FUEL_VOLTAGE";
  static final String LOWER_RESIST_FUEL_PREFERENCE_NT = "Kicker/LOWER_RESIST_FUEL_VOLTAGE";

  static final TalonFXConfiguration UPPER_KICKER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(8))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(
                      InvertedValue
                          .Clockwise_Positive)); // needs 8 rotations of the motor to rotate the
  // upper kicker once

  static final TalonFXConfiguration LOWER_KICKER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(
                      InvertedValue
                          .Clockwise_Positive)); // needs 2 rotations of the motor to rotate the
  // lower kicker once

  static final double UPPER_MOTOR_GEARING = 8;
  static final double LOWER_MOTOR_GEARING = 2;

  static final double UPPER_SHOOT_VOLTAGE = 7;
  static final double LOWER_SHOOT_VOLTAGE = 7;

  static final double UPPER_RESIST_FUEL_VOLTAGE = -3;
  static final double LOWER_RESIST_FUEL_VOLTAGE = -3;
}
