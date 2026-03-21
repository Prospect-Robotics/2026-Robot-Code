package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO extends AutoCloseable {
  @AutoLog
  class KickerIOInputs {
    public Voltage motor1Voltage = Volts.of(0);
    public AngularVelocity motor1RotationalVelocity = RotationsPerSecond.of(0);
    public Current motor1Current = Amps.of(0);

    public Voltage motor2Voltage = Volts.of(0);
    public AngularVelocity motor2RotationalVelocity = RotationsPerSecond.of(0);
    public Current motor2Current = Amps.of(0);
  }

  @Override
  default void close() {}

  default void setMotorVoltage(Voltage kickerMotorVoltage) {}

  default void updateState(KickerIOInputs inputs) {}
}
