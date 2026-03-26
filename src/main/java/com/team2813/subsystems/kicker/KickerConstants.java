package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.MomentOfInertia;

class KickerConstants {
  static final MomentOfInertia KICKER_SIM_MOI = KilogramSquareMeters.of(0.0000535531);
  static final String SHOOT_PREFERENCE_NT = "Kicker/SHOOT_VOLTAGE";
  static final String RESIST_FUEL_PREFERENCE_NT = "Kicker/RESIST_FUEL_VOLTAGE";

  static final TalonFXConfiguration UPPER_KICKER_MOTOR_CONFIG =
          new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(8))
                  .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)); // needs 8 rotations of the motor to rotate the upper kicker once

  static final TalonFXConfiguration LOWER_KICKER_MOTOR_CONFIG =
          new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2))
                  .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)); // needs 2 rotations of the motor to rotate the lower kicker once

  static final double UPPER_MOTOR_GEARING = 8;
  static final double LOWER_MOTOR_GEARING = 2;

  static final double SHOOT_VOLTAGE = 7;
  static final double RESIST_FUEL_VOLTAGE = -3;
}
