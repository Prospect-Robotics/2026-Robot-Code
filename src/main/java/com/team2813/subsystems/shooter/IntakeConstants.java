package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
  public static final Voltage INTAKE_VOLTAGE = Volts.of(3);
  public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-3);

  public static final TalonFXConfiguration MAIN_INTAKE_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
}
