package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class HopperConstants {

  static {
    Preferences.initFloat("Hopper/ROLLER_INTAKE_VOLTAGE", 5);
    Preferences.initDouble("Hopper/ROLLER_OUTTAKE_VOLTAGE", -5);
    Preferences.initFloat("Hopper/FEEDER_INTAKE_VOLTAGE", 3);
    Preferences.initDouble("Hopper/FEEDER_OUTTAKE_VOLTAGE", -3);
  }

  public static Voltage getRollerIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/ROLLER_INTAKE_VOLTAGE", 5)); // 5 is the backup.
  }

  public static Voltage getRollerOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/ROLLER_OUTTAKE_VOLTAGE", -5));
  }

  public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static Voltage getFeederIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/FEEDER_INTAKE_VOLTAGE", 3));
  }

  public static Voltage getFeederOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/FEEDER_OUTTAKE_VOLTAGE", -3));
  }

  public static final TalonFXConfiguration MAIN_FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
}
