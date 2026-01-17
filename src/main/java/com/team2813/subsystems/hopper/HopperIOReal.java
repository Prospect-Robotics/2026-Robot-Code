package com.team2813.subsystems.hopper;

import static com.team2813.subsystems.hopper.HopperConstants.HOTDOG_ROLLER_MOTOR_CONFIG;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
  private TalonFX rollerMotor;

  public HopperIOReal() {
    rollerMotor = new TalonFX(Constants.ROLLER_MOTOR_CAN_ID);
    rollerMotor.getConfigurator().apply(HOTDOG_ROLLER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValue();
    inputs.rollerMotorVoltage = rollerMotor.getMotorVoltage().getValue();
    inputs.rollerMotorRPS = rollerMotor.getRotorVelocity().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    rollerMotor.setVoltage(voltage.in(Volts));
  }
}
