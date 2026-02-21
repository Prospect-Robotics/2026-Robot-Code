package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
  private final TalonFX mainRollerMotor; // Top magazine motor.
  private final TalonFX followerRollerMotor; // Bottom magazine motor.

  private final TalonFX feederMotor; // Runs the indexer.

  public HopperIOReal() {
    mainRollerMotor = new TalonFX(Constants.MAIN_ROLLER_MOTOR_CAN_ID);
    mainRollerMotor.getConfigurator().apply(HopperConstants.ROLLER_MOTOR_CONFIG);

    followerRollerMotor = new TalonFX(Constants.FOLLOWER_ROLLER_MOTOR_CAN_ID);
    // Motors are on opposite sides of the magazine.
    followerRollerMotor.setControl(
        new Follower(Constants.MAIN_ROLLER_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));

    feederMotor = new TalonFX(Constants.FEEDER_MOTOR_ID);
    feederMotor.getConfigurator().apply(HopperConstants.RIGHT_FEEDER_MOTOR_CONFIG);
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    inputs.mainRollerMotorVoltage = mainRollerMotor.getMotorVoltage().getValue();
    inputs.mainRollerMotorRPS = mainRollerMotor.getRotorVelocity().getValue();
    inputs.mainRollerMotorCurrent = mainRollerMotor.getStatorCurrent().getValue();

    inputs.followerRollerMotorVoltage = followerRollerMotor.getMotorVoltage().getValue();
    inputs.followerRollerMotorRPS = followerRollerMotor.getRotorVelocity().getValue();
    inputs.followerRollerMotorCurrent = followerRollerMotor.getStatorCurrent().getValue();

    inputs.feederVoltage = feederMotor.getMotorVoltage().getValue();
    inputs.feederRPS = feederMotor.getRotorVelocity().getValue();
    inputs.feederCurrent = feederMotor.getStatorCurrent().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    mainRollerMotor.setVoltage(rollerVoltage.in(Volts));
    feederMotor.setVoltage(feederVoltage.in(Volts));
  }
}
