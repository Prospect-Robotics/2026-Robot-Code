package com.team2813.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;

public class IntakeConstants {

  public static final double INTAKE_MOTOR_TO_INTAKE_GEARING = 2.5; // reduction

  public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(INTAKE_MOTOR_TO_INTAKE_GEARING));

  public static final double EXTENDER_MOTOR_TO_EXTENDER_GEARING = 3; // reduction

  public static final TalonFXConfiguration EXTENDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKP(2).withKI(0.001).withKD(0.000))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(EXTENDER_MOTOR_TO_EXTENDER_GEARING));

  public static final double PULLEY_RADIUS_METERS = 0.0889;

  public static final Distance INCHES_PER_ROTATION = Inches.of(Math.PI);

  public enum
      ExtenderPositions { // added 0.2 in the direction of motion to prevent stalling and ensure
    // the intake retracts all the way back
    OUT(Inches.of(10.95)),
    IN(Inches.of(-0.2));

    private final Distance position;

    ExtenderPositions(Distance of) {
      this.position = of;
    }

    public Distance getPosition() {
      return position;
    }
  }

  static {
    Preferences.initFloat("Intake/INTAKE_MOTOR_VOLTAGE", 5);
    Preferences.initFloat("Intake/OUTTAKE_MOTOR_VOLTAGE", -5);
  }

  public static Voltage getIntakeMotorVoltage() {
    return Volts.of(Preferences.getDouble("Intake/INTAKE_MOTOR_VOLTAGE", 5));
  }

  public static Voltage getOuttakeMotorVoltage() {
    return Volts.of(Preferences.getDouble("Intake/OUTTAKE_MOTOR_VOLTAGE", -5));
  }

  //  public static Voltage getExtenderInVoltage() {
  //    return Volts.of(Preferences.getDouble("Intake/EXTENDER_IN_VOLTAGE", 5));
  //  }
  //
  //  public static Voltage getExtenderOutVoltage() {
  //    return Volts.of(Preferences.getDouble("Intake/EXTENDER_OUT_VOLTAGE", -5));
  //  }

  public static Angle getExtendOutSetpoint() {
    return Rotations.of(ExtenderPositions.OUT.getPosition().div(INCHES_PER_ROTATION).magnitude());
  }

  public static Angle getExtendInSetpoint() {
    return Rotations.of(ExtenderPositions.IN.getPosition().div(INCHES_PER_ROTATION).magnitude());
  }

  public static final double MANUAL_SPEED_FACTOR =
      3.0; // controls how fast the extension moves during manual control
}
