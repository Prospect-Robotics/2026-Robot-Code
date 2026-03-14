package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2813.Constants;
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
    Preferences.initDouble(SHOOTER_TRENCH_SHOOT_PREFERENCE_NT, 150);
    Preferences.initDouble(SHOOTER_HUB_SHOOT_PREFERENCE_NT, 60);
    Preferences.initDouble(SHOOTER_HERD_SHOOT_PREFERENCE_NT, 115);
    Preferences.initDouble(SHOOTER_OUTTAKE_PREFERENCE_NT, -5);
  }

  // Reminder: this is the right shooter motor when robot is viewed from behind.
  public static final TalonFXConfiguration MAIN_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(
              new Slot0Configs().withKS(0.099892).withKV(0.115).withKA(0.0020241).withKP(0.026743))
          .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(60)));

  public static final TalonFXConfiguration FOLLOWER_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs().withKS(0.099892).withKV(0.115).withKA(0.0020241).withKP(0.026743))
          .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60)));
  // Left shooter motor.
  public static final Follower FOLLOWER_SHOOTER_CONTROL_MODE =
      new Follower(Constants.MAIN_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed);

  public static final double SHOOTER_MOTOR_TO_FLYWHEEL_GEARING = 1.0;

  public static AngularVelocity getShooterTrenchShootVelocity() {
    return RotationsPerSecond.of(Preferences.getDouble(SHOOTER_TRENCH_SHOOT_PREFERENCE_NT, 90));
  }

  public static AngularVelocity getShooterHubShootVelocity() {
    return RotationsPerSecond.of(Preferences.getDouble(SHOOTER_HUB_SHOOT_PREFERENCE_NT, 115));
  }

  public static AngularVelocity getShooterHerdShootVelocity() {
    return RotationsPerSecond.of(Preferences.getDouble(SHOOTER_HERD_SHOOT_PREFERENCE_NT, 60));
  }

  public static Voltage getShooterOuttakeVoltage() {
    return Volts.of(Preferences.getDouble(SHOOTER_OUTTAKE_PREFERENCE_NT, -5));
  }
}
