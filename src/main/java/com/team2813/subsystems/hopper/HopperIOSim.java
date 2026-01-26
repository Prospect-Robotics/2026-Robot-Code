package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOSim implements HopperIO {
  private TalonFX mainRollerMotor;
  private TalonFXSimState mainRollerMotorSimState;

  private TalonFX followerRollerMotor;
  private TalonFXSimState followerRollerMotorSimState;

  // Left motor when seen from the front (intake side).
  private TalonFX rightFeederMotor;
  private TalonFXSimState rightFeederMotorSimState;

  // Right motor when seen from the front (intake side).
  private TalonFX leftFeederMotor; // Right motor when seen from the front (intake side).
  private TalonFXSimState leftFeederMotorSimState;

  public HopperIOSim() {
    mainRollerMotor = new TalonFX(Constants.MAIN_ROLLER_MOTOR_CAN_ID);
    mainRollerMotor.getConfigurator().apply(HopperConstants.ROLLER_MOTOR_CONFIG);
    mainRollerMotorSimState = mainRollerMotor.getSimState();

    followerRollerMotor = new TalonFX(Constants.FOLLOWER_ROLLER_MOTOR_CAN_ID);
    followerRollerMotor.setControl(
        new Follower(Constants.MAIN_ROLLER_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));
    followerRollerMotorSimState = followerRollerMotor.getSimState();

    rightFeederMotor = new TalonFX(Constants.RIGHT_FEEDER_MOTOR_ID);
    rightFeederMotor.getConfigurator().apply(HopperConstants.RIGHT_FEEDER_MOTOR_CONFIG);
    rightFeederMotorSimState = rightFeederMotor.getSimState();

    leftFeederMotor = new TalonFX(Constants.LEFT_FEEDER_MOTOR_ID);
    leftFeederMotor.getConfigurator().apply(HopperConstants.LEFT_FEEDER_MOTOR_CONFIG);
    leftFeederMotorSimState = leftFeederMotor.getSimState();
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    mainRollerMotorSimState.setSupplyVoltage(Volts.of(12));
    followerRollerMotorSimState.setSupplyVoltage(Volts.of(12));
    rightFeederMotorSimState.setSupplyVoltage(Volts.of(12));
    leftFeederMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.mainRollerMotorVoltage = mainRollerMotor.getMotorVoltage().getValue();
    inputs.mainRollerMotorRPS = mainRollerMotor.getRotorVelocity().getValue();
    inputs.mainRollerMotorCurrent = mainRollerMotor.getStatorCurrent().getValue();

    inputs.followerRollerMotorVoltage = followerRollerMotor.getMotorVoltage().getValue();
    inputs.followerRollerMotorRPS = followerRollerMotor.getRotorVelocity().getValue();
    inputs.followerRollerMotorCurrent = followerRollerMotor.getStatorCurrent().getValue();

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
    mainRollerMotor.setVoltage(rollerVoltage.in(Volts));
    rightFeederMotor.setVoltage(rightFeederVoltage.in(Volts));
    leftFeederMotor.setVoltage(leftFeederVoltage.in(Volts));
  }
}
