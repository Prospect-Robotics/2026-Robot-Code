package com.team2813.subsystems.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private Angle motorSetpoint = Rotations.of(0);

  public HoodIOReal() {
    motor = new TalonFX(Constants.HOOD_MOTOR_ID);
    motor.getConfigurator().apply(HoodConstants.PIVOT_MOTOR_CONFIG);
  }

  @Override
  public void updateState(HoodIOInputs inputs) {
    inputs.motorVoltage = motor.getMotorVoltage().getValue();
    inputs.motorVelocity = motor.getVelocity().getValue();
    inputs.motorStatorCurrent = motor.getStatorCurrent().getValue();
    inputs.motorSupplyCurrent = motor.getSupplyCurrent().getValue();
    inputs.motorSetpoint = motorSetpoint;
    inputs.motorAngle = motor.getPosition().getValue();
  }

  @Override
  public void setSetpoint(Angle angle) {
    motorSetpoint = angle;
    motor.setControl(positionVoltage.withPosition(angle));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void close() {
    motor.close();
  }
}
