package com.team2813.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

class HoodConstants {
  static final TalonFXConfiguration PIVOT_MOTOR_CONFIG = new TalonFXConfiguration();

  static final String HUB_ANGLE_PREFERENCE = "Hub/hubAngle";
  static final double DEFAULT_HUB_ANGLE = 45;
  static final String TRENCH_ANGLE_PREFERENCE = "Hub/trenchAngle";
  static final double DEFUALT_TRENCH_ANGLE = 30;

  private HoodConstants() {
    throw new AssertionError("Not Instantiable!");
  }
}
