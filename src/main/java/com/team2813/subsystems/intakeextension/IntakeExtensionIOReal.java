package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import com.team2813.subsystems.Simulation;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeExtensionIOReal implements IntakeExtensionIO {
  private final TalonFX extenderMotor;
  private Angle extensionSetpoint = Rotation.of(0);

  public IntakeExtensionIOReal() {
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    extenderMotor.getConfigurator().apply(IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG);
    extenderMotor.setPosition(Rotation.of(0)); // intake should be fully retracted on bootup
  }

  @Override
  public void updateState(IntakeExtensionIOInputs inputs) {
    inputs.extenderMotorVoltage = extenderMotor.getMotorVoltage().getValue();
    inputs.extenderMotorRPS = extenderMotor.getRotorVelocity().getValue();
    inputs.extenderMotorCurrent = extenderMotor.getStatorCurrent().getValue();
    inputs.extenderMotorPosition = extenderMotor.getPosition().getValue();
    inputs.extenderMotorSetpoint = extensionSetpoint;
  }

  @Override
  public void setExtensionSetpoint(Angle setpoint) {
    extensionSetpoint = setpoint;
    extenderMotor.setControl(new PositionVoltage(setpoint));
  }

  @Override
  public void setExtenderVoltage(Voltage extenderVoltage) {
    extenderMotor.setVoltage(extenderVoltage.in(Volts));
  }

  TalonFXSimState getExtenderMotorSimState() {
    return Simulation.getConfiguredSimState(extenderMotor, TalonFXSimState.MotorType.KrakenX44);
  }

  @Override
  public void close() {
    extenderMotor.close();
  }
}
