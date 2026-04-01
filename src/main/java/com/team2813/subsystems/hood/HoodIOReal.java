package com.team2813.subsystems.hood;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Angle;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();

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
    inputs.motorAngle = motor.getPosition().getValue();
  }

  @Override
  public void setSetpoint(Angle angle) {
    motor.setControl(positionVoltage.withPosition(angle));
  }

  @Override
  public void neutral() {
    motor.setControl(neutralOut);
  }

  @Override
  public void close() {
    motor.close();
  }
}
