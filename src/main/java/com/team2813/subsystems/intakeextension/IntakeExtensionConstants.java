package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import org.littletonrobotics.junction.Logger;

public class IntakeExtensionConstants {

  public static final double EXTENDER_MOTOR_TO_EXTENDER_GEARING = 3; // reduction

  public static final Mass WEIGHT_OF_EXTENDER_CARRIAGE =
      Kilograms.of(6.137); // Value taken from CAD

  public static final TalonFXConfiguration EXTENDER_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKP(2).withKI(0.001).withKD(0.000))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1));

  public static final Distance PULLEY_RADIUS = Inches.of(0.5);

  public static final double DISTANCE_METERS_TO_MOTOR_ROTATIONS =
      IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING
          / (2.0 * Math.PI * IntakeExtensionConstants.PULLEY_RADIUS.in(Meters));

  public static final Distance EXTENDED_POSITION = Inches.of(10.75);
  public static final Distance RETRACTED_POSITION = Inches.of(0);

  public static final Distance ANTI_STALL_DISTANCE = Inches.of(0.2);

  public enum ExtenderPositions {
    // Added 0.2 in the direction of motion to prevent stalling and ensure the intake retracts all
    // the way back.

    OUT(EXTENDED_POSITION.plus(ANTI_STALL_DISTANCE)),
    IN(RETRACTED_POSITION.minus(ANTI_STALL_DISTANCE));

    private final Distance position;

    ExtenderPositions(Distance of) {
      this.position = of;
    }

    public Distance getPosition() {
      return position;
    }
  }

  public static Angle getExtendOutSetpoint() {
    Logger.recordOutput("IntakeExtension/Setpoint", ExtenderPositions.OUT);
    return Rotations.of(
        ExtenderPositions.OUT.getPosition().in(Meters) * DISTANCE_METERS_TO_MOTOR_ROTATIONS);
  }

  public static Angle getExtendInSetpoint() {
    Logger.recordOutput("IntakeExtension/Setpoint", ExtenderPositions.IN);
    return Rotations.of(
        ExtenderPositions.IN.getPosition().in(Meters) * DISTANCE_METERS_TO_MOTOR_ROTATIONS);
  }

  // controls how fast the extension moves during manual control
  public static final double MANUAL_SPEED_FACTOR = 3.0;
}
