package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

  @AutoLog
  class IntakeRollerIOInputs {
    public Voltage intakeMotorVoltage = Volts.of(0);
    public AngularVelocity intakeMotorRPS = RotationsPerSecond.of(0);
    public Current intakeMotorCurrent = Amps.of(0);
  }

  default void updateState(IntakeRollerIOInputs inputs) {}

  default void setIntakeMotorVoltage(Voltage intakeMotorVoltage) {}
}
