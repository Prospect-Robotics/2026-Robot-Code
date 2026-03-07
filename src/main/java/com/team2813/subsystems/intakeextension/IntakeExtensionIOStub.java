package com.team2813.subsystems.intakeextension;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** No-nothing implementation of IntakeExtensionIO; used for replay mode. */
public final class IntakeExtensionIOStub implements IntakeExtensionIO {

  @Override
  public void updateState(IntakeExtensionIOInputs inputs) {}

  @Override
  public void setExtenderVoltage(Voltage extensionVoltage) {}

  @Override
  public void setExtensionSetpoint(Angle setpoint) {}

  @Override
  public void close() {}
}
