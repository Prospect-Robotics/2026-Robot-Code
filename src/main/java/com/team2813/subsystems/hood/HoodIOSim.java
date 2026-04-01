package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  private final TalonFX motor;
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final SingleJointedArmSim hoodSim;

  public HoodIOSim() {
    motor = new TalonFX(Constants.HOOD_MOTOR_ID);
    motor.getConfigurator().apply(HoodConstants.PIVOT_MOTOR_CONFIG);
    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 8, 0.0529, 8.982529, 0.284256, 0.685682, true, 0.284256);
  }

  @Override
  public void updateState(HoodIOInputs inputs) {
    inputs.motorVoltage = motor.getMotorVoltage().getValue();
    hoodSim.setInputVoltage(inputs.motorVoltage.in(Volts));

    hoodSim.update(Constants.SIM_TIME_PERIOD);

    var simState = motor.getSimState();
    simState.setRawRotorPosition(Radians.of(hoodSim.getAngleRads()).times(8));
    simState.setRotorVelocity(RadiansPerSecond.of(hoodSim.getVelocityRadPerSec()).times(8));
    simState.setSupplyVoltage(Volts.of(12));

    inputs.motorVelocity = motor.getVelocity().getValue();
    inputs.motorStatorCurrent = motor.getStatorCurrent().getValue();
    inputs.motorSupplyCurrent = motor.getSupplyCurrent().getValue();
    inputs.motorAngle = motor.getPosition().getValue();
  }

  @Override
  public void setSetpoint(Angle angle) {
    motor.setControl(positionVoltage);
  }

  @Override
  public void neutral() {
    motor.setControl(neutralOut);
  }

  @Override
  public void close() {
    motor.close();
  }
}
