package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtensionIO extends AutoCloseable {

  @AutoLog
  class IntakeExtensionIOInputs {
    public Voltage extenderMotorVoltage = Volts.of(0);
    public AngularVelocity extenderMotorRPS = RotationsPerSecond.of(0);
    public Current extenderMotorCurrent = Amps.of(0);
    public Angle extenderMotorPosition = Rotation.of(0);
    public Angle extenderMotorSetpoint = Rotation.of(0);
  }

  void updateState(IntakeExtensionIOInputs inputs);

  void setExtenderVoltage(Voltage extensionVoltage);

  void setExtensionSetpoint(Angle setpoint);

  @Override
  void close();
}
