package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOSim implements HopperIO {
  private TalonFX rollerMotor;
  private TalonFXSimState rollerMotorSimState;

  // Left motor when seen from the front (intake side).
  private TalonFX rightFeederMotor;
  private TalonFXSimState rightFeederMotorSimState;

  // Right motor when seen from the front (intake side).
  private TalonFX leftFeederMotor; // Right motor when seen from the front (intake side).
  private TalonFXSimState leftFeederMotorSimState;

  public HopperIOSim() {
    rollerMotor = new TalonFX(Constants.MAIN_ROLLER_MOTOR_CAN_ID);
    rollerMotor.getConfigurator().apply(HopperConstants.ROLLER_MOTOR_CONFIG);
    rollerMotorSimState = rollerMotor.getSimState();

    rightFeederMotor = new TalonFX(Constants.RIGHT_FEEDER_MOTOR_ID);
    rightFeederMotor.getConfigurator().apply(HopperConstants.RIGHT_FEEDER_MOTOR_CONFIG);
    rightFeederMotorSimState = rightFeederMotor.getSimState();

    leftFeederMotor = new TalonFX(Constants.LEFT_FEEDER_MOTOR_ID);
    leftFeederMotor.getConfigurator().apply(HopperConstants.LEFT_FEEDER_MOTOR_CONFIG);
    leftFeederMotorSimState = leftFeederMotor.getSimState();
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    rollerMotorSimState.setSupplyVoltage(Volts.of(12));
    rightFeederMotorSimState.setSupplyVoltage(Volts.of(12));
    leftFeederMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValue();
    inputs.rollerMotorVoltage = rollerMotor.getMotorVoltage().getValue();
    inputs.rollerMotorRPS = rollerMotor.getRotorVelocity().getValue();
    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValue();

    inputs.rightFeederVoltage = rightFeederMotor.getMotorVoltage().getValue();
    inputs.rightFeederRPS = rightFeederMotor.getRotorVelocity().getValue();
    inputs.rightFeederCurrent = rightFeederMotor.getStatorCurrent().getValue();

    inputs.leftFeederVoltage = leftFeederMotor.getMotorVoltage().getValue();
    inputs.leftFeederRPS = leftFeederMotor.getRotorVelocity().getValue();
    inputs.leftFeederCurrent = leftFeederMotor.getStatorCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(
      Voltage rollerVoltage, Voltage rightFeederVoltage, Voltage leftFeederVoltage) {
    rollerMotor.setVoltage(rollerVoltage.in(Volts));
    rightFeederMotor.setVoltage(rightFeederVoltage.in(Volts));
    leftFeederMotor.setVoltage(leftFeederVoltage.in(Volts));
  }
}
