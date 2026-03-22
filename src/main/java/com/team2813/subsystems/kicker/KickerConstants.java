package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.MomentOfInertia;

class KickerConstants {
  static final MomentOfInertia KICKER_SIM_MOI = KilogramSquareMeters.of(0.0000535531);
  static final String SHOOT_PREFERENCE_NT = "Kicker/SHOOT_VOLTAGE";
  static final String RESIST_FUEL_PREFERENCE_NT = "Kicker/RESIST_FUEL_VOLTAGE";

  static final TalonFXConfiguration KICKER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(35)));

  static final TalonFXConfiguration KICKER_MOTOR_2_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(35)));

  static final double KICKER_MOTOR_TO_FLYWHEEL_GEARING = 2.0 / 5.0;

  static final double SHOOT_VOLTAGE = 7;
  static final double RESIST_FUEL_VOLTAGE = -3;
}
