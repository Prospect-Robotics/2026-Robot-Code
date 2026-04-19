package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

public class HoodConstants {
  public static final double HOOD_GEAR_RATIO = 128;
  public static final MomentOfInertia HOOD_MOI = KilogramSquareMeters.of(0.0529);
  public static final Angle MINIMUM_SHOOTER_ANGLE = Degrees.of(0);
  public static final Angle MAXIMUM_SHOOTER_ANGLE = Degrees.of(22);
  public static final Distance SIM_ARM_LENGTH = Centimeter.of(10);

  /**
   * After trying to move the hood for this many seconds, the command will timeout and stop the
   * motor where it is.
   */
  static final Time HOOD_MOVEMENT_TIMEOUT = Seconds.of(3);

  /** The accepted difference between the motor angle and its setpoint. */
  static final Angle ACCEPTABLE_MOTOR_ERROR = Rotations.of(0.4);

  static final TalonFXConfiguration PIVOT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          // TODO: Run sysid to get PID values
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.2)
                  .withKS(0.22349)
                  .withKV(0.0081471)
                  .withKA(0.00044251)
                  .withKG(0.15395))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  static final String HUB_ANGLE_PREFERENCE = "Hood/HUB_HOOD_ANGLE_DEGREES";
  static final double DEFAULT_HUB_ANGLE_DEGREES = 17;
  static final String TRENCH_ANGLE_PREFERENCE = "Hood/TRENCH_HOOD_ANGLE_DEGREES";
  static final double DEFAULT_TRENCH_ANGLE_DEGREES = 40;

  private HoodConstants() {
    throw new AssertionError("Not Instantiable!");
  }
}
