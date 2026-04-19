package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class ShooterConstants {

  public static final String SHOOTER_TRENCH_SHOOT_PREFERENCE_NT =
      "Shooter/SHOOTER_TRENCH_SHOOT_VELOCITY_RPS";
  public static final String SHOOTER_HUB_SHOOT_PREFERENCE_NT =
      "Shooter/SHOOTER_HUB_SHOOT_VELOCITY_RPS";

  public static final String SHOOTER_HERD_SHOOT_PREFERENCE_NT =
      "Shooter/SHOOTER_HERD_SHOOT_VELOCITY_RPS";

  public static final String SHOOTER_OUTTAKE_PREFERENCE_NT = "Shooter/SHOOTER_OUTTAKE_VOLTAGE";

  public static final double SHOOTER_SIM_MOI = 0.00303431; // in kilograms*meters squared.

  /**
   * Because shooting a ton of fuel will slow the flywheel, we will aim about a 1/4 of a hub farther
   * than the center of the hub to counteract this speed loss.
   */
  public static final Distance EXTRA_HUB_AIMING_DISTANCE = Inches.of(10.425);

  /**
   * Used for automatically running kicker and hopper while shooting. If the motor speed is within
   * this value of Rot/s of the setpoint, {@link Shooter#isMotorVelocityWithinTolerance()} will
   * return true.
   */
  public static final AngularVelocity SHOOTER_SPOOL_SPEED_TOLERANCE = RotationsPerSecond.of(2);

  static {
    // Shooter motors.
    Preferences.initDouble(SHOOTER_TRENCH_SHOOT_PREFERENCE_NT, 100);
    Preferences.initDouble(SHOOTER_HUB_SHOOT_PREFERENCE_NT, 60);
    Preferences.initDouble(SHOOTER_HERD_SHOOT_PREFERENCE_NT, 55);
    Preferences.initDouble(SHOOTER_OUTTAKE_PREFERENCE_NT, -5);
  }

  static final Slot0Configs MOTORS_SLOT0_CONFIG =
      new Slot0Configs().withKS(0.22432).withKV(0.12688).withKA(0.010285).withKP(0.076732);

  // Reminder: this is the upper right shooter motor when robot is viewed from behind.
  public static final TalonFXConfiguration UPPER_RIGHT_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(MOTORS_SLOT0_CONFIG)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(70))
                  .withSupplyCurrentLimit(50));

  // Reminder: this is the lower right shooter motor.
  public static final TalonFXConfiguration LOWER_RIGHT_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(MOTORS_SLOT0_CONFIG)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(70))
                  .withSupplyCurrentLimit(50));

  // Reminder: this is the upper left shooter motor when the robot is viewed from behind.
  public static final TalonFXConfiguration UPPER_LEFT_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(MOTORS_SLOT0_CONFIG)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(80))
                  .withSupplyCurrentLimit(50));

  // Reminder: this is the lower left shooter motor.
  public static final TalonFXConfiguration LOWER_LEFT_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(MOTORS_SLOT0_CONFIG)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(80))
                  .withSupplyCurrentLimit(50));

  public static final double SHOOTER_MOTOR_TO_FLYWHEEL_GEARING = 1.0;

  public static AngularVelocity getShooterTrenchShootVelocity() {
    return RotationsPerSecond.of(Preferences.getDouble(SHOOTER_TRENCH_SHOOT_PREFERENCE_NT, 100));
  }

  public static AngularVelocity getShooterHubShootVelocity() {
    return RotationsPerSecond.of(Preferences.getDouble(SHOOTER_HUB_SHOOT_PREFERENCE_NT, 60));
  }

  public static AngularVelocity getShooterHerdShootVelocity() {
    return RotationsPerSecond.of(Preferences.getDouble(SHOOTER_HERD_SHOOT_PREFERENCE_NT, 55));
  }

  public static Voltage getShooterOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(SHOOTER_OUTTAKE_PREFERENCE_NT, -5));
  }
}
