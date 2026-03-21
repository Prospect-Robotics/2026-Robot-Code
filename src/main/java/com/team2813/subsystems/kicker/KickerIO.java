package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO extends AutoCloseable {
  @AutoLog
  class KickerIOInputs {
    public Voltage motorVoltage = Volts.of(0);
    public AngularVelocity motorRotationalVelocity = RotationsPerSecond.of(0);
    public Current motorStatorCurrent = Amps.of(0);
    public Current motorSupplyCurrent = Amps.of(0);
  }

  @Override
  default void close() {}

  default void setMotorVoltage(Voltage kickerMotorVoltage) {}

  default void updateState(KickerIOInputs inputs) {}
}
