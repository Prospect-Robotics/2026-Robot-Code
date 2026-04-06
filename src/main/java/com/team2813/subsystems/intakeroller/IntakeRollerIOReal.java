package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class IntakeRollerIOReal implements IntakeRollerIO {
  private TalonFX intakeMotor;

  public IntakeRollerIOReal() {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);
    intakeMotor.getConfigurator().apply(IntakeRollerConstants.LEFT_INTAKE_MOTOR_CONFIG);
  }

  @Override
  public void updateState(IntakeRollerIOInputs inputs) {
    inputs.leftIntakeMotorVoltage = intakeMotor.getMotorVoltage().getValue();
    inputs.leftIntakeMotorRPS = intakeMotor.getVelocity().getValue();
    inputs.leftIntakeMotorStatorCurrent = intakeMotor.getStatorCurrent().getValue();
    inputs.leftIntakeMotorSupplyCurrent = intakeMotor.getSupplyCurrent().getValue();
  }

  @Override
  public void setIntakeMotorVoltage(Voltage intakeMotorVoltage) {
    intakeMotor.setVoltage(intakeMotorVoltage.in(Volts));
  }
}
