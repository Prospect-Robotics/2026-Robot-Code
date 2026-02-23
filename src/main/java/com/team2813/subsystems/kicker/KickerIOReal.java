package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class KickerIOReal implements KickerIO {
  private final TalonFX motor;

  public KickerIOReal() {
    motor = new TalonFX(Constants.KICKER_MOTOR_ID);
    motor.getConfigurator().apply(KickerConstants.KICKER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(KickerIOInputs inputs) {
    inputs.motorVoltage = motor.getMotorVoltage().getValue();
    inputs.motorRotationalVelocity = motor.getVelocity().getValue();
    inputs.motorCurrent = motor.getStatorCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage kickerMotorVoltage) {
    motor.setVoltage(kickerMotorVoltage.in(Volts));
  }

  @Override
  public void close() {
    motor.close();
  }
}
