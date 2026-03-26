package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO extends AutoCloseable {
  @AutoLog
  class KickerIOInputs {
    public Voltage upperMotorVoltage = Volts.of(0);
    public AngularVelocity upperMotorRotationalVelocity = RotationsPerSecond.of(0);
    public Current upperMotorStatorCurrent = Amps.of(0);
    public Current upperMotorSupplyCurrent = Amps.of(0);

    public Voltage lowerMotorVoltage = Volts.of(0);
    public AngularVelocity lowerMotorRotationalVelocity = RotationsPerSecond.of(0);
    public Current lowerMotorStatorCurrent = Amps.of(0);
    public Current lowerMotorSupplyCurrent = Amps.of(0);
  }

  @Override
  default void close() {}

  default void setMotorVoltage(Voltage upperKickerMotorVoltage, Voltage lowerKickerMotorVoltage) {}

  default void updateState(KickerIOInputs inputs) {}
}
