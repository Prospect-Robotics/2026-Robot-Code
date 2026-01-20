package com.team2813.subsystems.hopper;

import static com.team2813.subsystems.hopper.HopperConstants.HOTDOG_ROLLER_MOTOR_CONFIG;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOSim implements HopperIO {
  private TalonFX rollerMotor;
  private TalonFXSimState rollerMotorSimState;

  public HopperIOSim() {
    rollerMotor = new TalonFX(Constants.ROLLER_MOTOR_CAN_ID);
    rollerMotor.getConfigurator().apply(HOTDOG_ROLLER_MOTOR_CONFIG);
    rollerMotorSimState = rollerMotor.getSimState();
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    rollerMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValue();
    inputs.rollerMotorVoltage = rollerMotor.getMotorVoltage().getValue();
    inputs.rollerMotorRPS = rollerMotor.getRotorVelocity().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    rollerMotor.setVoltage(voltage.in(Volts));
  }
}
