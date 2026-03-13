package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class HopperConstants {

  public static final double ROLLER_SIM_MOI = 0.00057684; // in kilograms*meters squared.

  static {
    // Roller motors.
    Preferences.initFloat("Hopper/ROLLER_INTAKE_VOLTAGE", 6.5f);
    Preferences.initFloat("Hopper/ROLLER_OUTTAKE_VOLTAGE", -8);

    // Feeder/Vectoring motors.
    Preferences.initFloat("Hopper/RIGHT_FEEDER_INTAKE_VOLTAGE", 6.5f);
    Preferences.initFloat("Hopper/RIGHT_FEEDER_OUTTAKE_VOLTAGE", -8);
  }

  // Roller Motor Configs
  public static Voltage getRollerIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/ROLLER_INTAKE_VOLTAGE", 8)); // 5 is the backup.
  }

  public static Voltage getRollerOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/ROLLER_OUTTAKE_VOLTAGE", -8));
  }

  public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  // TODO: Change this later to the actual number.
  public static final double ROLLER_MOTOR_TO_ROLLER_GEARING = 1;

  // Feeder Motor Configs
  public static Voltage getFeederIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/FEEDER_INTAKE_VOLTAGE", 8));
  }

  public static Voltage getFeederOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/FEEDER_OUTTAKE_VOLTAGE", -8));
  }

  public static final TalonFXConfiguration RIGHT_FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
