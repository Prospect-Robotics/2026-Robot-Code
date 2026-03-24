package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
  private final TalonFX mainFeederMotor; // Top magazine motor.
  private final TalonFX followerFeederMotor; // Bottom magazine motor.

  private final TalonFX indexerMotor; // Runs the indexer.

  public HopperIOReal() {
    mainFeederMotor = new TalonFX(Constants.MAIN_FEEDER_MOTOR_CAN_ID);
    mainFeederMotor.getConfigurator().apply(HopperConstants.MAIN_ROLLER_MOTOR_CONFIG);

    followerFeederMotor = new TalonFX(Constants.FOLLOWER_FEEDER_MOTOR_CAN_ID);
    followerFeederMotor.getConfigurator().apply(HopperConstants.FOLLOWER_FEEDER_MOTOR_CONFIG);

    indexerMotor = new TalonFX(Constants.INDEXER_MOTOR_ID);
    indexerMotor.getConfigurator().apply(HopperConstants.INDEXER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    inputs.mainFeederMotorVoltage = mainFeederMotor.getMotorVoltage().getValue();
    inputs.mainFeederMotorRPS = mainFeederMotor.getRotorVelocity().getValue();
    inputs.mainFeederMotorStatorCurrent = mainFeederMotor.getStatorCurrent().getValue();
    inputs.mainFeederMotorSupplyCurrent = mainFeederMotor.getSupplyCurrent().getValue();

    inputs.followerFeederMotorVoltage = followerFeederMotor.getMotorVoltage().getValue();
    inputs.followerFeederMotorRPS = followerFeederMotor.getRotorVelocity().getValue();
    inputs.followerFeederMotorStatorCurrent = followerFeederMotor.getStatorCurrent().getValue();
    inputs.followerFeederMotorSupplyCurrent = followerFeederMotor.getSupplyCurrent().getValue();

    inputs.indexerMotorVoltage = indexerMotor.getMotorVoltage().getValue();
    inputs.indexerMotorRPS = indexerMotor.getRotorVelocity().getValue();
    inputs.indexerMotorStatorCurrent = indexerMotor.getStatorCurrent().getValue();
    inputs.indexerMotorSupplyCurrent = indexerMotor.getSupplyCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    mainFeederMotor.setVoltage(rollerVoltage.in(Volts));
    followerFeederMotor.setVoltage(feederVoltage.in(Volts));
    indexerMotor.setVoltage(feederVoltage.in(Volts));
  }
}
