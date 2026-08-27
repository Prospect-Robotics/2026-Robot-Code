package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class HopperConstants {

  public static final double FEEDER_SIM_MOI = 0.00057684; // in kilograms*meters squared.

  public static final String FEEDER_INTAKE_VOLTAGE_NT = "Hopper/FEEDER_INTAKE_VOLTAGE";
  public static final String FEEDER_OUTTAKE_VOLTAGE_NT = "Hopper/FEEDER_OUTTAKE_VOLTAGE";

  public static final String INDEXER_INTAKE_VOLTAGE_NT = "Hopper/INDEXER_INTAKE_VOLTAGE";
  public static final String INDEXER_OUTTAKE_VOLTAGE_NT = "Hopper/INDEXER_OUTTAKE_VOLTAGE";

  static {
    // Feeder motors.
    Preferences.initDouble(FEEDER_INTAKE_VOLTAGE_NT, 8);
    Preferences.initDouble(FEEDER_OUTTAKE_VOLTAGE_NT, -6);

    // Indexer/Vectoring motors.
    Preferences.initDouble(INDEXER_INTAKE_VOLTAGE_NT, 8);
    Preferences.initDouble(INDEXER_OUTTAKE_VOLTAGE_NT, -6);
  }

  // Roller Motor Configs
  public static Voltage getFeederIntakeVoltage() {
    return Volts.of(Preferences.getDouble(FEEDER_INTAKE_VOLTAGE_NT, 6.5)); // 5 is the backup.
  }

  public static Voltage getFeederOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(FEEDER_OUTTAKE_VOLTAGE_NT, -6));
  }

  // The motor on the bottom feeders.
  public static final TalonFXConfiguration FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(30))
                  .withStatorCurrentLimit(Amps.of(60)))
          .withSlot0(new Slot0Configs().withKS(0.2).withKV(0.116));

  // TODO: Change this later to the actual number.
  public static final double FEEDER_MOTOR_TO_ROLLER_GEARING = 1;

  // Feeder Motor Configs
  public static Voltage getIndexerIntakeVoltage() {
    return Volts.of(Preferences.getDouble(INDEXER_INTAKE_VOLTAGE_NT, 5));
  }

  public static Voltage getIndexerOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(INDEXER_OUTTAKE_VOLTAGE_NT, -6));
  }

  public static final TalonFXConfiguration INDEXER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(30))
                  .withStatorCurrentLimit(Amps.of(60)));
}
