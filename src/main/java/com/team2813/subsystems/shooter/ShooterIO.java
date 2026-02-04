package com.team2813.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public Voltage mainShooterMotorVoltage = Volts.of(0);
        public AngularVelocity mainShooterMotorRPS = RotationsPerSecond.of(0);
        public Current mainShooterMotorCurrent = Amps.of(0);

        public Voltage followerShooterMotorVoltage = Volts.of(0);
        public AngularVelocity followerShooterMotorRPS = RotationsPerSecond.of(0);
        public Current followerShooterMotorCurrent = Amps.of(0);

        public Voltage kickerShooterMotorVoltage = Volts.of(0);
        public AngularVelocity kickerShooterMotorRPS = RotationsPerSecond.of(0);
        public Current kickerShooterMotorCurrent = Amps.of(0);
    }

    /**
     * Updates Advantage kit autologged input data, as well as any other necessary states (like in
     * sim)
     *
     * @param inputs The "struct" (data class) to handle hardware inputs.
     */
    default void updateState(ShooterIOInputs inputs) {}

    default void setMotorVoltage() {}
}
