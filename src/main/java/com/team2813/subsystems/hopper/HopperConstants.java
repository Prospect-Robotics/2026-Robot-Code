package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class HopperConstants {

  static {
    // roller motors.
    Preferences.initFloat("Hopper/ROLLER_INTAKE_VOLTAGE", 5);
    Preferences.initFloat("Hopper/ROLLER_OUTTAKE_VOLTAGE", -5);

    // feeder/vector motors.
    Preferences.initFloat("Hopper/RIGHT_FEEDER_INTAKE_VOLTAGE", 3);
    Preferences.initFloat("Hopper/RIGHT_FEEDER_OUTTAKE_VOLTAGE", -3);

    Preferences.initFloat("Hopper/LEFT_FEEDER_INTAKE_VOLTAGE", 3);
    Preferences.initFloat("Hopper/LEFT_FEEDER_OUTTAKE_VOLTAGE", -3);
  }

  // Roller Motor Configs
  public static Voltage getRollerIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/ROLLER_INTAKE_VOLTAGE", 5)); // 5 is the backup.
  }

  public static Voltage getRollerOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/ROLLER_OUTTAKE_VOLTAGE", -5));
  }

  public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  // TODO: Change this later because the design lead fell asleep as of writing this and I made the number up.
  public static final double ROLLER_MOTOR_TO_ROLLER_GEARING = 1;


  // Feeder Motor Configs
  public static Voltage getRightFeederIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/RIGHT_FEEDER_INTAKE_VOLTAGE", 3));
  }

  public static Voltage getRightFeederOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/RIGHT_FEEDER_OUTTAKE_VOLTAGE", -3));
  }

  public static Voltage getLeftFeederIntakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/LEFT_FEEDER_INTAKE_VOLTAGE", 3));
  }

  public static Voltage getLeftFeederOuttakeVoltage() {
    return Volts.of(Preferences.getDouble("Hopper/LEFT_FEEDER_OUTTAKE_VOLTAGE", -3));
  }

  public static final TalonFXConfiguration RIGHT_FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

  public static final TalonFXConfiguration LEFT_FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
