package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

  @AutoLog
  class IntakeRollerIOInputs {
    public Voltage leftIntakeMotorVoltage = Volts.of(0);
    public AngularVelocity leftIntakeMotorRPS = RotationsPerSecond.of(0);
    public Current leftIntakeMotorStatorCurrent = Amps.of(0);
    public Current leftIntakeMotorSupplyCurrent = Amps.of(0);

    public Voltage rightIntakeMotorVoltage = Volts.of(0);
    public AngularVelocity rightIntakeMotorRPS = RotationsPerSecond.of(0);
    public Current rightIntakeMotorStatorCurrent = Amps.of(0);
    public Current rightIntakeMotorSupplyCurrent = Amps.of(0);
  }

  default void updateState(IntakeRollerIOInputs inputs) {}

  default void setIntakeMotorVoltage(Voltage intakeMotorVoltage) {}
}
