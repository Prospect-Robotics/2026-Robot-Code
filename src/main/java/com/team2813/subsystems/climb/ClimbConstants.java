package com.team2813.subsystems.climb;

// import static edu.wpi.first.units.Units.Millimeters;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {
  // Gearing ratio = (36t:12t) * (40t:20t) * (36t:24t) = 9:1
  // Gear chains looked up from the CAD model of the climb gearbox.
  public static final double INNER_MOTOR_TO_CLIMB_GEARING = 9;
  public static final double OUTER_MOTOR_TO_CLIMB_GEARING = 9;

  public static final TalonFXConfiguration INNER_MOTOR_TO_CLIMB_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              // TODO(stefan): Tune these values on the real robot.
              // Currently, they look good in Sim.
              new Slot0Configs().withKS(0.25).withKV(0.05).withKP(1.0).withKI(0.0).withKD(0.1))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  public static final TalonFXConfiguration OUTER_MOTOR_TO_CLIMB_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              // TODO(stefan): Tune these values on the real robot.
              // Currently, they look good in Sim.
              new Slot0Configs().withKS(0.25).withKV(0.05).withKP(1.0).withKI(0.0).withKD(0.1))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  // NOTE: These may be the same value, because it seems both motors have the same gearbox.
  // The spool for the climb is just a hex shaft with size of 1/2" (distance between two opposite
  // flat surfaces).
  // This corresponds to sqrt(3) circumference of the hex shaft. I.e., one full rotation of the
  // shaft
  // spools sqrt(3)" worth of cable.
  // Due to 9:1 gearbox reduction, one rotation of the motor produces 1/9 of a shaft rotation, or
  // sqrt(3)/9 inches of climb height change.
  public static final Distance INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION =
      Inches.of(Math.sqrt(3) / INNER_MOTOR_TO_CLIMB_GEARING);
  public static final Distance OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION =
      Inches.of(Math.sqrt(3) / OUTER_MOTOR_TO_CLIMB_GEARING);

  public static final Distance INNER_CLIMB_SPOOL_RADIUS = Inches.of(0.25);
  public static final Distance OUTER_CLIMB_SPOOL_RADIUS = Inches.of(0.25);

  public static final Distance INNER_CLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance INNER_CLIMB_MID_HEIGHT = Inches.of(14.0);
  public static final Distance INNER_CLIMB_MAX_HEIGHT = Inches.of(28.0);

  public static final Distance OUTER_CLIMB_MIN_HEIGHT = Inches.of(0);
  public static final Distance OUTER_CLIMB_MID_HEIGHT = Inches.of(14.0);
  public static final Distance OUTER_CLIMB_MAX_HEIGHT = Inches.of(28.0);

  public static final Voltage MANUAL_INNER_CLIMB_VOLTAGE = Volts.of(3);

  public static final Angle CLIMB_SETPOINT_TO_MOTOR_ROT_TOLERANCE = Rotations.of(2);
}
