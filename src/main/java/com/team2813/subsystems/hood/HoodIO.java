package com.team2813.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO extends AutoCloseable {
  @AutoLog
  class HoodIOInputs {
    public Voltage motorVoltage;
    public AngularVelocity motorVelocity;
    public Current motorStatorCurrent;
    public Current motorSupplyCurrent;
    public Angle motorAngle;
    public Angle motorSetpoint;
  }

  default void updateState(HoodIOInputs inputs) {}

  default void setSetpoint(Angle angle) {}

  default void neutral() {}

  @Override
  default void close() {}
}
