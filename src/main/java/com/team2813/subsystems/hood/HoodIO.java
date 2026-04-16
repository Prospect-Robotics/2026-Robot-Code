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
   * Sets the angle to bring the shooter to. This angle is from the hard stop to the hood position;
   * higher angle values are a flatter angle
   *
   * @param angle The shooter angle
   */
  default void setSetpoint(Angle angle) {}

  default void neutral() {}

  @Override
  default void close() {}
}
