package com.team2813.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {}

    /**
     * Updates Advantage kit autologged input data, as well as any other necessary states (like in
     * sim)
     *
     * @param inputs The "struct" (data class) to handle hardware inputs.
     */
    default void updateState(ShooterIOInputs inputs) {}

    default void setMotorVoltage() {}
}
