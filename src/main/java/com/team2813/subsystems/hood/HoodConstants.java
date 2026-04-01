package com.team2813.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

class HoodConstants {
  static final TalonFXConfiguration PIVOT_MOTOR_CONFIG = new TalonFXConfiguration();

  private HoodConstants() {
    throw new AssertionError("Not Instantiable!");
  }
}
