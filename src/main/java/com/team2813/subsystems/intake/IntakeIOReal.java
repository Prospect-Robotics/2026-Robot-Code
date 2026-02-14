package com.team2813.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFX extenderMotor;

  public IntakeIOReal() {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    // Apply any necessary configuration to the intakeMotor here
    intakeMotor.getConfigurator().apply(IntakeConstants.INTAKE_MOTOR_CONFIG);
    extenderMotor.getConfigurator().apply(IntakeConstants.EXTENDER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(IntakeIOInputs inputs) {
    inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValue();
    inputs.intakeMotorRPS = intakeMotor.getRotorVelocity().getValue();
    inputs.intakeMotorCurrent = intakeMotor.getStatorCurrent().getValue();

    inputs.extenderMotorVoltage = extenderMotor.getMotorVoltage().getValue();
    inputs.extenderMotorRPS = extenderMotor.getRotorVelocity().getValue();
    inputs.extenderMotorCurrent = extenderMotor.getStatorCurrent().getValue();
    inputs.extenderMotorPosition = extenderMotor.getPosition().getValue();
    inputs.extenderMotorSetpoint = extenderMotor.getClosedLoopReference(false).getValue();
  }

  @Override
  public void setIntakeVoltage(Voltage intakeVoltage) {
    intakeMotor.setVoltage(intakeVoltage.in(edu.wpi.first.units.Units.Volts));
  }

  @Override
  public void setExtensionSetpoint(Angle setpoint) {
    extenderMotor.setControl(new PositionVoltage(setpoint));
  }

  @Override
  public void stopExtender() {
    extenderMotor.setVoltage(0);
  }
}
