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
    // Motors are on opposite sides of the magazine.
    //    followerRollerMotor.setControl(
    //        new Follower(Constants.MAIN_ROLLER_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));

    indexerMotor = new TalonFX(Constants.INDEXER_MOTOR_ID);
    indexerMotor.getConfigurator().apply(HopperConstants.INDEXER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    inputs.mainFeederMotorVoltage = mainFeederMotor.getMotorVoltage().getValue();
    inputs.mainFeederMotorRPS = mainFeederMotor.getRotorVelocity().getValue();
    inputs.mainFeederMotorCurrent = mainFeederMotor.getStatorCurrent().getValue();

    inputs.followerFeederMotorVoltage = followerFeederMotor.getMotorVoltage().getValue();
    inputs.followerFeederMotorRPS = followerFeederMotor.getRotorVelocity().getValue();
    inputs.followerFeederMotorCurrent = followerFeederMotor.getStatorCurrent().getValue();

    inputs.indexerVoltage = indexerMotor.getMotorVoltage().getValue();
    inputs.indexerRPS = indexerMotor.getRotorVelocity().getValue();
    inputs.indexerCurrent = indexerMotor.getStatorCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    mainFeederMotor.setVoltage(rollerVoltage.in(Volts));
    followerFeederMotor.setVoltage(feederVoltage.in(Volts));
    indexerMotor.setVoltage(feederVoltage.in(Volts));
  }
}
