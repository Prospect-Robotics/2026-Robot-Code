package com.team2813.subsystems.hood;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

class HoodConstants {
  // TODO: Get real value!
  static final double HOOD_GEAR_RATIO = 128;
  static final MomentOfInertia HOOD_MOI = Units.KilogramSquareMeters.of(0.0529);
  static final Angle MINIMUM_SHOOTER_ANGLE = Units.Degrees.of(0);
  static final Angle MAXIMUM_SHOOTER_ANGLE = Units.Degrees.of(90);
  static final Distance SIM_ARM_LENGTH = Units.Centimeter.of(10);

  static final TalonFXConfiguration PIVOT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          // TODO: Run sysid to get PID values
          .withSlot0(new Slot0Configs().withKP(5).withKI(0).withKD(0).withKS(0).withKA(0))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  static final String HUB_ANGLE_PREFERENCE = "Hood/hubAngle";
  static final double DEFAULT_HUB_ANGLE_DEGREES = 45;
  static final String TRENCH_ANGLE_PREFERENCE = "Hood/trenchAngle";
  static final double DEFAULT_TRENCH_ANGLE_DEGREES = 30;

  private HoodConstants() {
    throw new AssertionError("Not Instantiable!");
  }
}
