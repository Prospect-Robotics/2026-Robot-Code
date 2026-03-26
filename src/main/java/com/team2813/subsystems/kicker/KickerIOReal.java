package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class KickerIOReal implements KickerIO {
  private final TalonFX upperKickerMotor;
  private final TalonFX lowerKickerMotor;

  public KickerIOReal() {
    upperKickerMotor = new TalonFX(Constants.UPPER_KICKER_MOTOR_ID);
    upperKickerMotor.getConfigurator().apply(KickerConstants.UPPER_KICKER_MOTOR_CONFIG);

    lowerKickerMotor = new TalonFX(Constants.LOWER_KICKER_MOTOR_ID);
    lowerKickerMotor.getConfigurator().apply(KickerConstants.LOWER_KICKER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(KickerIOInputs inputs) {
    inputs.upperMotorVoltage = upperKickerMotor.getMotorVoltage().getValue();
    inputs.upperMotorRotationalVelocity = upperKickerMotor.getVelocity().getValue();
    inputs.upperMotorStatorCurrent = upperKickerMotor.getStatorCurrent().getValue();
    inputs.upperMotorSupplyCurrent = upperKickerMotor.getSupplyCurrent().getValue();

    inputs.lowerMotorVoltage = lowerKickerMotor.getMotorVoltage().getValue();
    inputs.lowerMotorRotationalVelocity = lowerKickerMotor.getVelocity().getValue();
    inputs.lowerMotorStatorCurrent = lowerKickerMotor.getStatorCurrent().getValue();
    inputs.lowerMotorSupplyCurrent = lowerKickerMotor.getSupplyCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage upperKickerMotorVoltage, Voltage lowerKickerMotorVoltage) {
    upperKickerMotor.setVoltage(upperKickerMotorVoltage.in(Volts));
    lowerKickerMotor.setVoltage(lowerKickerMotorVoltage.in(Volts));
  }

  @Override
  public void close() {
    upperKickerMotor.close();
    lowerKickerMotor.close();
  }
}
