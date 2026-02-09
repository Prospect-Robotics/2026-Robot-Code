package com.team2813.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public Voltage extenderMotorVoltage = Volts.of(0);
    public AngularVelocity extenderMotorRPS = RotationsPerSecond.of(0);
    public Current extenderMotorCurrent = Amps.of(0);

    public Voltage intakeMotorVoltage = Volts.of(0);
    public AngularVelocity intakeMotorRPS = RotationsPerSecond.of(0);
    public Current intakeMotorCurrent = Amps.of(0);
  }

  default void updateState(IntakeIOInputs inputs) {}

  default void setMotorVoltage(Voltage intakeVoltage, Voltage extenderVoltage) {}
}
