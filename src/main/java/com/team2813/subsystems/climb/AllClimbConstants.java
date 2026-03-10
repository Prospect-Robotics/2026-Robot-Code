package com.team2813.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Distance;

public record AllClimbConstants(
    String climbName,
    double motorToClimbGearing,
    TalonFXConfiguration motorToClimbConfig,
    Distance climbHeightChangePerRotation,
    Distance climbSpoolRadius,
    Distance climbMinHeight,
    Distance climbMaxHeight) {}
