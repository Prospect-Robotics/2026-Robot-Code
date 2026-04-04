package com.team2813.subsystems.hood;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

class HoodConstants {
  // TODO: Get real value!
  static final double HOOD_GEAR_RATIO = 8 * 14.5;
  static final MomentOfInertia HOOD_MOI = Units.KilogramSquareMeters.of(0.0529);
  static final Angle MINIMUM_SHOOTER_ANGLE = Units.Radians.of(0.284256);
  static final Angle MAXIMUM_SHOOTER_ANGLE = Units.Radians.of(0.685682);
  static final Distance SHOOTER_RADIUS = Units.Meters.of(8.982529);
  static final TalonFXConfiguration PIVOT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          // TODO: Run sysid to get PID values
          .withSlot0(new Slot0Configs().withKP(1).withKI(0).withKD(0).withKS(0).withKA(0))
          .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(HOOD_GEAR_RATIO));

  static final String HUB_ANGLE_PREFERENCE = "Hub/hubAngle";
  static final double DEFAULT_HUB_ANGLE = 45;
  static final String TRENCH_ANGLE_PREFERENCE = "Hub/trenchAngle";
  static final double DEFAULT_TRENCH_ANGLE = 30;

  private HoodConstants() {
    throw new AssertionError("Not Instantiable!");
  }
}
