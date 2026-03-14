package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

class IntakeExtensionConstants {

  public static final double EXTENDER_MOTOR_TO_EXTENDER_GEARING = 3; // reduction

  public static final Mass WEIGHT_OF_EXTENDER_CARRIAGE =
      Kilograms.of(6.137); // Value taken from CAD

  public static final TalonFXConfiguration EXTENDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              // Values chosen after some consultation with Gemini:
              // https://share.google/aimode/Ha33a7FUqS9EhAzI4
              new Slot0Configs().withKS(0.25).withKV(0.25).withKP(10).withKI(0.0).withKD(0.1))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
          .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(60)));

  public static final Distance PULLEY_RADIUS = Inches.of(0.5);

  public static final double DISTANCE_METERS_TO_MOTOR_ROTATIONS =
      EXTENDER_MOTOR_TO_EXTENDER_GEARING / (2.0 * Math.PI * PULLEY_RADIUS.in(Meters));

  public static final Distance EXTENDED_POSITION = Inches.of(10.75);
  public static final Distance RETRACTED_POSITION = Inches.of(0);

  public static final Distance ANTI_STALL_DISTANCE = Inches.of(0.25);

  // TODO: Migrate this enum to a more suitable location than a Constants Class.
  public enum ExtenderPositions {
    // Added 0.2 in the direction of motion to prevent stalling and ensure the intake retracts all
    // the way back.

    OUT(EXTENDED_POSITION.plus(ANTI_STALL_DISTANCE)),
    // About halfway in, used for walle mode, as we retract to this before fully extending.
    MIDDLE(Inches.of(5.4)),
    IN(RETRACTED_POSITION.minus(ANTI_STALL_DISTANCE));

    private final Distance position;

    ExtenderPositions(Distance of) {
      this.position = of;
    }

    public Angle getAngle() {
      return Rotations.of(position.in(Meters) * DISTANCE_METERS_TO_MOTOR_ROTATIONS);
    }
  }

  // TODO: Migrate this methods to a more suitable location than a Constants Class.
  public static Distance toIntakeExtensionPosition(Angle motorPosition) {
    return Meters.of(motorPosition.in(Rotations) / DISTANCE_METERS_TO_MOTOR_ROTATIONS);
  }

  // Controls how fast the extension moves during manual control
  public static final double MANUAL_SPEED_FACTOR = 3.0;

  private IntakeExtensionConstants() {
    throw new AssertionError("Not instantiable");
  }
}
