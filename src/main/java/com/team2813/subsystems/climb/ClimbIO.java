package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ClimbIO {
  class ClimbIOInputs {
    public Voltage leftClimbMotorVoltage = Volts.of(0);
    public AngularVelocity leftClimbMotorRPS = RotationsPerSecond.of(0);
    public Current leftClimbMotorCurrent = Amps.of(0);

    public Voltage rightClimbMotorVoltage = Volts.of(0);
    public AngularVelocity rightClimbMotorRPS = RotationsPerSecond.of(0);
    public Current rightClimbMotorCurrent = Amps.of(0);
  }

  default void updateState(ClimbIOInputs inputs) {}

  default void setMotorVoltage(Voltage climbMotor1Voltage, Voltage climbMotor2Voltage) {}
}
