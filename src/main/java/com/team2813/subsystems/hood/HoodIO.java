package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO extends AutoCloseable {
  @AutoLog
  class HoodIOInputs {
    public Voltage motorVoltage = Volts.of(0);
    public AngularVelocity motorVelocity = RadiansPerSecond.of(0);
    public Current motorStatorCurrent = Amps.of(0);
    public Current motorSupplyCurrent = Amps.of(0);
    public Angle motorAngle = Radians.of(0);
    public Angle motorSetpoint = Radians.of(0);
  }

  default void updateState(HoodIOInputs inputs) {}

  /**
   * Used for sysID
   *
   * @param voltage
   */
  default void setVoltage(Voltage voltage) {}

  default void setSetpoint(Angle angle) {}

  default void stop() {}

  @Override
  default void close() {}
}
