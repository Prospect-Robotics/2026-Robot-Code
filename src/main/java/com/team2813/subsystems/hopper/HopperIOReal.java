package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
  private TalonFX rollerMotor;
  private TalonFX mainFeederMotor; // Left motor when seen from the front (intake side).
  private TalonFX followerFeederMotor; // Right motor when seen from the front (intake side).

  public HopperIOReal() {
    rollerMotor = new TalonFX(Constants.ROLLER_MOTOR_CAN_ID);
    rollerMotor.getConfigurator().apply(HopperConstants.ROLLER_MOTOR_CONFIG);

    mainFeederMotor = new TalonFX(Constants.MAIN_FEEDER_MOTOR_ID);
    mainFeederMotor.getConfigurator().apply(HopperConstants.MAIN_FEEDER_MOTOR_CONFIG);

    followerFeederMotor = new TalonFX(Constants.FOLLOWER_FEEDER_MOTOR_ID);
    followerFeederMotor.setControl(
        new Follower(Constants.MAIN_FEEDER_MOTOR_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValue();
    inputs.rollerMotorRPS = rollerMotor.getRotorVelocity().getValue();
    inputs.rollerMotorVoltage = rollerMotor.getMotorVoltage().getValue();

    inputs.mainFeederVoltage = mainFeederMotor.getMotorVoltage().getValue();
    inputs.mainFeederRPS = mainFeederMotor.getRotorVelocity().getValue();
    inputs.mainFeederCurrent = mainFeederMotor.getStatorCurrent().getValue();

    inputs.followerFeederVoltage = followerFeederMotor.getMotorVoltage().getValue();
    inputs.followerFeederRPS = followerFeederMotor.getRotorVelocity().getValue();
    inputs.followerFeederCurrent = followerFeederMotor.getStatorCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    rollerMotor.setVoltage(rollerVoltage.in(Volts));
    mainFeederMotor.setVoltage(feederVoltage.in(Volts));
  }
}
