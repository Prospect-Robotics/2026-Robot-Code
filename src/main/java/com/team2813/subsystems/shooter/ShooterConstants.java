package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class ShooterConstants {

  public static final String SHOOTER_INTAKE_PREFERENCE_NT = "Shooter/SHOOTER_INTAKE_VOLTAGE";
  public static final String SHOOTER_OUTTAKE_PREFERENCE_NT = "Shooter/SHOOTER_OUTTAKE_VOLTAGE";
  public static final String KICKER_INTAKE_PREFERENCE_NT = "Shooter/KICKER_INTAKE_VOLTAGE";
  public static final String KICKER_OUTTAKE_PREFERENCE_NT = "Shooter/KICKER_OUTTAKE_VOLTAGE";

  public static final double SHOOTER_SIM_MOI = 0.00303431; // in kilograms*meters squared.
  public static final double KICKER_SIM_MOI = 0.0000535531; // in kilograms*meters squared.

  static {
    // Shooter motors.
    Preferences.initDouble(SHOOTER_INTAKE_PREFERENCE_NT, 11);
    Preferences.initDouble(SHOOTER_OUTTAKE_PREFERENCE_NT, -5);

    // Kicker motors.
    Preferences.initDouble(KICKER_INTAKE_PREFERENCE_NT, 5);
    Preferences.initDouble(KICKER_OUTTAKE_PREFERENCE_NT, -3);
  }

  // Reminder: this is the right shooter motor when robot is viewed from behind.
  public static final TalonFXConfiguration MAIN_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

  // Left shooter motor.
  public static final Follower FOLLOWER_SHOOTER_CONTROL_MODE =
      new Follower(Constants.MAIN_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed);

  public static final TalonFXConfiguration KICKER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static final double SHOOTER_MOTOR_TO_FLYWHEEL_GEARING = 1.0;

  public static final double KICKER_MOTOR_TO_FLYWHEEL_GEARING = 2.0 / 5.0;

  public static Voltage getShooterIntakeVoltage() {
    return Volts.of(Preferences.getDouble(SHOOTER_INTAKE_PREFERENCE_NT, 11));
  }

  public static Voltage getShooterOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(SHOOTER_OUTTAKE_PREFERENCE_NT, -5));
  }

  public static Voltage getKickerIntakeVoltage() {
    return Volts.of(Preferences.getDouble(KICKER_INTAKE_PREFERENCE_NT, 5));
  }

  public static Voltage getKickerOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(KICKER_OUTTAKE_PREFERENCE_NT, -3));
  }
}
