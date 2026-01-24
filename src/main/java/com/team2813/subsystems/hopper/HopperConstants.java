package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;

public class HopperConstants {
  // TODO: consider using Preferences or Smartdashboard to update voltage values w/o redeploy.
  private static final Voltage ROLLER_INTAKE_VOLTAGE = Volts.of(5);
  private static final Voltage ROLLER_OUTTAKE_VOLTAGE = Volts.of(-5);

  public static Voltage getROLLER_INTAKE_VOLTAGE() {
    return ROLLER_INTAKE_VOLTAGE;
  }

  public static Voltage getROLLER_OUTTAKE_VOLTAGE() {
    return ROLLER_OUTTAKE_VOLTAGE;
  }

  public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  private static final Voltage FEEDER_INTAKE_VOLTAGE = Volts.of(3);
  private static final Voltage FEEDER_OUTTAKE_VOLTAGE = Volts.of(-3);

  public static Voltage getFEEDER_OUTTAKE_VOLTAGE() {
    return FEEDER_OUTTAKE_VOLTAGE;
  }

  public static Voltage getFEEDER_INTAKE_VOLTAGE() {
    return FEEDER_INTAKE_VOLTAGE;
  }

  public static final TalonFXConfiguration MAIN_FEEDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

  public static void updatePreferences() {}
}
