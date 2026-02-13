package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ClimbIO {
     class ClimbIOInputs {
    public Voltage climbMotor1Voltage = Volts.of(0);
    public AngularVelocity climbMotor1RPS = RotationsPerSecond.of(0);
    public Current climbMotor1Current = Amps.of(0);

    public Voltage climbMotor2Voltage = Volts.of(0);
    public AngularVelocity climbMotor2RPS = RotationsPerSecond.of(0);
    public Current climbMotor2Current = Amps.of(0);
  }

  default void updateState(ClimbIOInputs inputs) {}

  default void setMotorVoltage(Voltage climbMotor1Voltage, Voltage climbMotor2Voltage) {}
}
