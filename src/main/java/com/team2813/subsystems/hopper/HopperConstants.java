package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class HopperConstants {

  public static final double ROLLER_SIM_MOI = 0.00057684; // in kilograms*meters squared.

  public static final String HOPPER_INTAKE_VOLTAGE_NT = "Hopper/ROLLER_INTAKE_VOLTAGE";
  public static final String HOPPER_OUTTAKE_VOLTAGE_NT = "Hopper/ROLLER_OUTTAKE_VOLTAGE";

  public static final String INDEXER_INTAKE_VOLTAGE_NT = "Hopper/FEEDER_INTAKE_VOLTAGE";
  public static final String INDEXER_OUTTAKE_VOLTAGE_NT = "Hopper/FEEDER_OUTTAKE_VOLTAGE";

  static {
    // Roller motors.
    Preferences.initDouble(HOPPER_INTAKE_VOLTAGE_NT, 6.5);
    Preferences.initDouble(HOPPER_OUTTAKE_VOLTAGE_NT, -8);

    // Feeder/Vectoring motors.
    Preferences.initDouble(INDEXER_INTAKE_VOLTAGE_NT, 5);
    Preferences.initDouble(INDEXER_OUTTAKE_VOLTAGE_NT, -8);
  }

  // Roller Motor Configs
  public static Voltage getRollerIntakeVoltage() {
    return Volts.of(Preferences.getDouble(HOPPER_INTAKE_VOLTAGE_NT, 6.5)); // 5 is the backup.
  }

  public static Voltage getRollerOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(HOPPER_OUTTAKE_VOLTAGE_NT, -8));
  }

  // TOP
  public static final TalonFXConfiguration MAIN_ROLLER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)); // 60 amps

  // Bottom Motor, opposite of main motor.
  public static final TalonFXConfiguration FOLLOWER_FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

  // TODO: Change this later to the actual number.
  public static final double ROLLER_MOTOR_TO_ROLLER_GEARING = 1;

  // Feeder Motor Configs
  public static Voltage getFeederIntakeVoltage() {
    return Volts.of(Preferences.getDouble(INDEXER_INTAKE_VOLTAGE_NT, 5));
  }

  public static Voltage getFeederOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(INDEXER_OUTTAKE_VOLTAGE_NT, -8));
  }

  public static final TalonFXConfiguration FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
