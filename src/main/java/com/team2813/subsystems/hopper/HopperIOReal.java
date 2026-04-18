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

  // Prep the fields we will log for each motor.
  // By having a status signal stored, we can refresh all the motor values at once.
  // Before, we would refresh all the motor values just to get one value, causing a performance
  // issue.
  StatusSignal<Voltage> mainFeederVoltage;
  StatusSignal<AngularVelocity> mainFeederRPS;
  StatusSignal<Current> mainFeederStatorCurrent; // Motor Control to Stator
  StatusSignal<Current> mainFeederSupplyCurrent; // Battery to Stator

  public HopperIOReal() {
    mainFeederMotor = new TalonFX(Constants.MAIN_FEEDER_MOTOR_CAN_ID);
    mainFeederMotor.getConfigurator().apply(HopperConstants.MAIN_ROLLER_MOTOR_CONFIG);

    mainFeederVoltage = mainFeederMotor.getMotorVoltage();
    mainFeederRPS = mainFeederMotor.getRotorVelocity();
    mainFeederStatorCurrent = mainFeederMotor.getStatorCurrent();
    mainFeederSupplyCurrent = mainFeederMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, mainFeederVoltage, mainFeederRPS, mainFeederStatorCurrent, mainFeederSupplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(mainFeederMotor);
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        mainFeederVoltage, mainFeederRPS, mainFeederStatorCurrent, mainFeederSupplyCurrent);

    inputs.mainFeederMotorVoltage = mainFeederVoltage.getValue();
    inputs.mainFeederMotorRPS = mainFeederRPS.getValue();
    inputs.mainFeederMotorStatorCurrent = mainFeederStatorCurrent.getValue();
    inputs.mainFeederMotorSupplyCurrent = mainFeederSupplyCurrent.getValue();
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    mainFeederMotor.setVoltage(rollerVoltage.in(Volts));
  }
}
