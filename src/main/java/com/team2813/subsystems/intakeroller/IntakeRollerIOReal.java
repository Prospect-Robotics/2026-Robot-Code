package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class IntakeRollerIOReal implements IntakeRollerIO {
  private TalonFX leftIntakeMotor;
  private TalonFX rightIntakeMotor;

  public IntakeRollerIOReal() {
    leftIntakeMotor = new TalonFX(Constants.LEFT_INTAKE_MOTOR_CAN_ID);
    leftIntakeMotor.getConfigurator().apply(IntakeRollerConstants.LEFT_INTAKE_MOTOR_CONFIG);

    rightIntakeMotor = new TalonFX(Constants.RIGHT_INTAKE_MOTOR_CAN_ID);
    rightIntakeMotor.getConfigurator().apply(IntakeRollerConstants.RIGHT_INTAKE_MOTOR_CONFIG);
  }

  @Override
  public void updateState(IntakeRollerIOInputs inputs) {
    inputs.leftIntakeMotorVoltage = leftIntakeMotor.getMotorVoltage().getValue();
    inputs.leftIntakeMotorRPS = leftIntakeMotor.getVelocity().getValue();
    inputs.leftIntakeMotorStatorCurrent = leftIntakeMotor.getStatorCurrent().getValue();
    inputs.leftIntakeMotorSupplyCurrent = leftIntakeMotor.getSupplyCurrent().getValue();

    inputs.rightIntakeMotorVoltage = rightIntakeMotor.getMotorVoltage().getValue();
    inputs.rightIntakeMotorRPS = rightIntakeMotor.getVelocity().getValue();
    inputs.rightIntakeMotorStatorCurrent = rightIntakeMotor.getStatorCurrent().getValue();
    inputs.rightIntakeMotorSupplyCurrent = rightIntakeMotor.getSupplyCurrent().getValue();
  }

  @Override
  public void setIntakeMotorVoltage(Voltage intakeMotorVoltage) {
    leftIntakeMotor.setVoltage(intakeMotorVoltage.in(Volts));
    rightIntakeMotor.setVoltage(intakeMotorVoltage.in(Volts));
  }
}
