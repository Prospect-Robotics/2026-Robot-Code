package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class KickerIOReal implements KickerIO {
  private final TalonFX motor1;
  private final TalonFX motor2;

  public KickerIOReal() {
    motor1 = new TalonFX(Constants.KICKER_MOTOR_1_ID);
    motor1.getConfigurator().apply(KickerConstants.KICKER_MOTOR_CONFIG);

    motor2 = new TalonFX(Constants.KICKER_MOTOR_2_ID);
    motor2.getConfigurator().apply(KickerConstants.KICKER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(KickerIOInputs inputs) {
    inputs.motor1Voltage = motor1.getMotorVoltage().getValue();
    inputs.motor1RotationalVelocity = motor1.getVelocity().getValue();
    inputs.motor1Current = motor1.getStatorCurrent().getValue();

    inputs.motor2Voltage = motor2.getMotorVoltage().getValue();
    inputs.motor2RotationalVelocity = motor2.getVelocity().getValue();
    inputs.motor2Current = motor2.getStatorCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage kickerMotorVoltage) {
    motor1.setVoltage(kickerMotorVoltage.in(Volts));
    motor2.setVoltage(kickerMotorVoltage.in(Volts));
  }

  @Override
  public void close() {
    motor1.close();
    motor2.close();
  }
}
