package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtensionIO extends AutoCloseable {

  @AutoLog
  class IntakeExtensionIOInputs {
    public Voltage extenderMotorVoltage = Volts.of(0);
    public AngularVelocity extenderMotorRPS = RotationsPerSecond.of(0);
    public Current extenderMotorStatorCurrent = Amps.of(0);
    public Current extenderMotorSupplyCurrent = Amps.of(0);
    public Angle extenderMotorPosition = Rotation.of(0);
    public Angle extenderMotorSetpoint = Rotation.of(0);
  }

  default void updateState(IntakeExtensionIOInputs inputs) {}

  default void setExtenderVoltage(Voltage extensionVoltage) {}

  default void setExtensionSetpoint(Angle setpoint) {}

  @Override
  default void close() throws Exception {}
}
