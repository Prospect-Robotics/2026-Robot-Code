package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  private final TalonFX motor;
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final SingleJointedArmSim hoodSim;

  public HoodIOSim() {
    motor = new TalonFX(Constants.HOOD_MOTOR_ID);
    motor.getConfigurator().apply(HoodConstants.PIVOT_MOTOR_CONFIG);
    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            HoodConstants.HOOD_GEAR_RATIO,
            HoodConstants.HOOD_MOI.in(KilogramSquareMeters),
            HoodConstants.SIM_ARM_LENGTH.in(Meters),
            HoodConstants.MINIMUM_SHOOTER_ANGLE.in(Radians),
            HoodConstants.MAXIMUM_SHOOTER_ANGLE.in(Radians),
            true,
            HoodConstants.MINIMUM_SHOOTER_ANGLE.in(Radians));
  }

  @Override
  public void updateState(HoodIOInputs inputs) {
    inputs.motorVoltage = motor.getMotorVoltage().getValue();
    hoodSim.setInputVoltage(inputs.motorVoltage.in(Volts));

    hoodSim.update(Constants.SIM_TIME_PERIOD);

    var simState = motor.getSimState();
    simState.setRawRotorPosition(
        Radians.of(hoodSim.getAngleRads()).times(HoodConstants.HOOD_GEAR_RATIO));
    simState.setRotorVelocity(
        RadiansPerSecond.of(hoodSim.getVelocityRadPerSec()).times(HoodConstants.HOOD_GEAR_RATIO));
    simState.setSupplyVoltage(Volts.of(12));

    SimulationVisualizer.getInstance().updateShooterHoodAngle(Radians.of(hoodSim.getAngleRads()));

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
    motor.stopMotor();
  }

  @Override
  public void close() {
    motor.close();
  }
}
