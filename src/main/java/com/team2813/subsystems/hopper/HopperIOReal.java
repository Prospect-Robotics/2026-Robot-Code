package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
  private final TalonFX mainFeederMotor; // Top magazine motor.
  private final TalonFX followerFeederMotor; // Bottom magazine motor.

  private final TalonFX indexerMotor; // Runs the indexer.

  // Prep the fields we will log for each motor.
  // By having a status signal stored, we can refresh all the motor values at once.
  // Before, we would refresh all the motor values just to get one value, causing a performance
  // issue.
  StatusSignal<Voltage> mainFeederVoltage;
  StatusSignal<AngularVelocity> mainFeederRPS;
  StatusSignal<Current> mainFeederStatorCurrent; // Motor Control to Stator
  StatusSignal<Current> mainFeederSupplyCurrent; // Battery to Stator

  StatusSignal<Voltage> followerFeederVoltage;
  StatusSignal<AngularVelocity> followerFeederRPS;
  StatusSignal<Current> followerFeederStatorCurrent;
  StatusSignal<Current> followerFeederSupplyCurrent;

  StatusSignal<Voltage> indexerVoltage;
  StatusSignal<AngularVelocity> indexerRPS;
  StatusSignal<Current> indexerStatorCurrent;
  StatusSignal<Current> indexerSupplyCurrent;

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

    mainFeederVoltage = mainFeederMotor.getMotorVoltage();
    mainFeederRPS = mainFeederMotor.getRotorVelocity();
    mainFeederStatorCurrent = mainFeederMotor.getStatorCurrent();
    mainFeederSupplyCurrent = mainFeederMotor.getSupplyCurrent();

    followerFeederVoltage = followerFeederMotor.getMotorVoltage();
    followerFeederRPS = followerFeederMotor.getRotorVelocity();
    followerFeederStatorCurrent = followerFeederMotor.getStatorCurrent();
    followerFeederSupplyCurrent = followerFeederMotor.getSupplyCurrent();

    indexerVoltage = indexerMotor.getMotorVoltage();
    indexerRPS = indexerMotor.getRotorVelocity();
    indexerStatorCurrent = indexerMotor.getStatorCurrent();
    indexerSupplyCurrent = indexerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        mainFeederVoltage,
        mainFeederRPS,
        mainFeederStatorCurrent,
        mainFeederSupplyCurrent,
        followerFeederVoltage,
        followerFeederRPS,
        followerFeederStatorCurrent,
        followerFeederSupplyCurrent,
        indexerVoltage,
        indexerRPS,
        indexerStatorCurrent,
        indexerSupplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(mainFeederMotor, followerFeederMotor, indexerMotor);
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        mainFeederVoltage,
        mainFeederRPS,
        mainFeederStatorCurrent,
        mainFeederSupplyCurrent,
        followerFeederVoltage,
        followerFeederRPS,
        followerFeederStatorCurrent,
        followerFeederSupplyCurrent,
        indexerVoltage,
        indexerRPS,
        indexerStatorCurrent,
        indexerSupplyCurrent);

    inputs.mainFeederMotorVoltage = mainFeederVoltage.getValue();
    inputs.mainFeederMotorRPS = mainFeederRPS.getValue();
    inputs.mainFeederMotorStatorCurrent = mainFeederStatorCurrent.getValue();
    inputs.mainFeederMotorSupplyCurrent = mainFeederSupplyCurrent.getValue();

    inputs.followerFeederMotorVoltage = followerFeederVoltage.getValue();
    inputs.followerFeederMotorRPS = followerFeederRPS.getValue();
    inputs.followerFeederMotorStatorCurrent = followerFeederStatorCurrent.getValue();
    inputs.followerFeederMotorSupplyCurrent = followerFeederSupplyCurrent.getValue();

    inputs.indexerMotorVoltage = indexerVoltage.getValue();
    inputs.indexerMotorRPS = indexerRPS.getValue();
    inputs.indexerMotorStatorCurrent = indexerStatorCurrent.getValue();
    inputs.indexerMotorSupplyCurrent = indexerSupplyCurrent.getValue();
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    mainFeederMotor.setVoltage(rollerVoltage.in(Volts));
    followerFeederMotor.setVoltage(feederVoltage.in(Volts));
    indexerMotor.setVoltage(feederVoltage.in(Volts));
  }
}
