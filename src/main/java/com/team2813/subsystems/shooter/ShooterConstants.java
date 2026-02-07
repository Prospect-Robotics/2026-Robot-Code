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

  static {
    // Shooter motors.
    Preferences.initDouble("Shooter/SHOOTER_INTAKE_VOLTAGE", 5);
    Preferences.initDouble("Shooter/SHOOTER_OUTTAKE_VOLTAGE", -5);

    // Kicker motors.
    Preferences.initDouble("Shooter/KICKER_INTAKE_VOLTAGE", 3);
    Preferences.initDouble("Shooter/KICKER_OUTTAKE_VOLTAGE", 3);
  }

  // Reminder: this is the left shooter motor when robot is viewed from behind.
  public static final TalonFXConfiguration MAIN_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  // Right shooter motor.
  public static final Follower FOLLOWER_SHOOTER_CONTROL_MODE =
      new Follower(Constants.MAIN_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed);

  public static final TalonFXConfiguration KICKER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static final double SHOOTER_MOTOR_TO_FLYWHEEL_GEARING = 1;

  public static final double KICKER_MOTOR_TO_FLYWHEEL_GEARING = 2.0 / 5.0;

  public static Voltage getShooterIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Shooter/SHOOTER_INTAKE_VOLTAGE", 5));
  }

  public static Voltage getShooterOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Shooter/SHOOTER_OUTTAKE_VOLTAGE", -5));
  }

  public static Voltage getKickerIntakeVoltage() {
    return Volts.of(Preferences.getDouble("KICKER_INTAKE_VOLTAGE", 3));
  }

  public static Voltage getKickerOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("KICKER_OUTTAKE_VOLTAGE", -3));
  }
}
