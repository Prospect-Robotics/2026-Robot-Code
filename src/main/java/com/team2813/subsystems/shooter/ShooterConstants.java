package com.team2813.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2813.Constants;

public class ShooterConstants {

  // Reminder: this is the left shooter motor when robot is viewed from behind.
  public static TalonFXConfiguration MAIN_SHOOTER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  // Right shooter motor.
  public static Follower FOLLOWER_SHOOTER_CONTROL_MODE =
      new Follower(Constants.MAIN_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed);

  public static TalonFXConfiguration KICKER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
