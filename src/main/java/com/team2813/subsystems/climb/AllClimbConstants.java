package com.team2813.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Distance;

/** Record class used for defining constants of different climb instances. */
record AllClimbConstants(
    String climbName,
    int climbCanID,
    double motorToClimbGearing,
    TalonFXConfiguration climbMotorConfig,
    Distance climbHeightChangePerRotation,
    Distance climbSpoolRadius,
    Distance climbMinHeight,
    Distance climbMidHeight,
    Distance climbMaxHeight) {}
