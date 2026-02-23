package com.team2813.subsystems.kicker;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

class KickerConstants {
  static final double KICKER_SIM_MOI = 0.0000535531; // in kilograms*meters squared.
  static final String SHOOT_PREFERENCE_NT = "Kicker/SHOOT_VOLTAGE";
  static final String RESIST_FUEL_PREFERENCE_NT = "Kicker/RESIST_FUEL_VOLTAGE";

  static final TalonFXConfiguration KICKER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  static final double KICKER_MOTOR_TO_FLYWHEEL_GEARING = 2.0 / 5.0;

  static final double SHOOT_VOLTAGE = 5;
  static final double RESIST_FUEL_VOLTAGE = -3;
}
