package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class IntakeRollerConstants {

  // NOTE: REMEMBER ALL DIRECTIONS ARE FROM WHEN THE ROBOT IS VIEWED FROM BEHIND.
  public static final double INTAKE_MOTOR_TO_INTAKE_GEARING = 2.5; // reduction

  public static final TalonFXConfiguration LEFT_INTAKE_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(50).withStatorCurrentLimit(60));

  public static final TalonFXConfiguration RIGHT_INTAKE_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withSupplyCurrentLimit(50).withStatorCurrentLimit(60));

  public static final String INTAKE_PREFERENCE_NT = "IntakeRoller/INTAKE_MOTOR_VOLTAGE";
  public static final String OUTTAKE_PREFERENCE_NT = "IntakeRoller/OUTTAKE_MOTOR_VOLTAGE";

  public static final double INTAKE_SIM_MOI = 0.00011331; // In kg*m^2

  static {
    Preferences.initFloat(INTAKE_PREFERENCE_NT, 9);
    Preferences.initFloat(OUTTAKE_PREFERENCE_NT, -9);
  }

  public static Voltage getIntakeVoltage() {
    return Volts.of(Preferences.getFloat(INTAKE_PREFERENCE_NT, 9));
  }

  public static Voltage getOuttakeVoltage() {
    return Volts.of(Preferences.getFloat(OUTTAKE_PREFERENCE_NT, -9));
  }
}
