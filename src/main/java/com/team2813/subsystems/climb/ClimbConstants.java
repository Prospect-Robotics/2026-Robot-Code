package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import java.util.function.Supplier;

public class ClimbConstants {
  public static final double CLIMB_MOTOR_1_GEARING = 9.0; // reduction

  public static final TalonFXConfiguration CLIMB_MOTOR_1_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(CLIMB_MOTOR_1_GEARING));

  public static final double CLIMB_MOTOR_2_GEARING = 9.0; // reduction

  public static final TalonFXConfiguration CLIMB_MOTOR_2_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(CLIMB_MOTOR_2_GEARING));

  public static Voltage getClimbMotor1Voltage() {
    return Volts.of(Preferences.getDouble("Climb/CLIMB_MOTOR_1_VOLTAGE", 5));
  }

  public static Voltage getClimbMotor2Voltage() {
    return Volts.of(Preferences.getDouble("Climb/CLIMB_MOTOR_2_VOLTAGE", -5));
  }

  public enum Position implements Supplier<Angle> {
    BOTTOM_OUT_HOOK(0),
    BOTTOM_IN_HOOK(0),
    TOP_IN_HOOK(0),
    TOP_OUT_HOOK(0),
    AUTO_IN_HOOK(0);

    private final Angle position;

    Position(double position) {
      this.position = Rotations.of(position);
    }

    @Override
    public Angle get() {
      return position;
    }
  }
}
