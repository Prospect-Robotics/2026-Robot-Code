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
  private final TalonFX feederMotor; // Top magazine motor.

  // Prep the fields we will log for each motor.
  // By having a status signal stored, we can refresh all the motor values at once.
  // Before, we would refresh all the motor values just to get one value, causing a performance
  // issue.
  StatusSignal<Voltage> feederVoltage;
  StatusSignal<AngularVelocity> feederRPS;
  StatusSignal<Current> feederStatorCurrent; // Motor Control to Stator
  StatusSignal<Current> feederSupplyCurrent; // Battery to Stator

  public HopperIOReal() {
    feederMotor = new TalonFX(Constants.FEEDER_MOTOR_CAN_ID);
    feederMotor.getConfigurator().apply(HopperConstants.FEEDER_MOTOR_CONFIG);

    feederVoltage = feederMotor.getMotorVoltage();
    feederRPS = feederMotor.getRotorVelocity();
    feederStatorCurrent = feederMotor.getStatorCurrent();
    feederSupplyCurrent = feederMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, feederVoltage, feederRPS, feederStatorCurrent, feederSupplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(feederMotor);
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(feederVoltage, feederRPS, feederStatorCurrent, feederSupplyCurrent);

    inputs.feederMotorVoltage = feederVoltage.getValue();
    inputs.feederMotorRPS = feederRPS.getValue();
    inputs.feederMotorStatorCurrent = feederStatorCurrent.getValue();
    inputs.feederMotorSupplyCurrent = feederSupplyCurrent.getValue();
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    feederMotor.setVoltage(rollerVoltage.in(Volts));
  }
}
