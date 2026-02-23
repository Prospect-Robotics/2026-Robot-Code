package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class KickerIOSim implements KickerIO {
  private final FlywheelSim flywheelSim;
  private final TalonFX motor;

  public KickerIOSim() {
    motor = new TalonFX(Constants.KICKER_MOTOR_ID);
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                KickerConstants.KICKER_SIM_MOI, // "Moment of Inertia" taken from OnShape.
                KickerConstants.KICKER_MOTOR_TO_FLYWHEEL_GEARING),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void setMotorVoltage(Voltage kickerMotorVoltage) {
    double volts = kickerMotorVoltage.in(Volts);
    motor.setVoltage(volts);
    flywheelSim.setInputVoltage(volts);
  }

  @Override
  public void updateState(KickerIOInputs inputs) {
    updateSimulation();

    inputs.motorVoltage = motor.getMotorVoltage().getValue();
    inputs.motorRotationalVelocity = motor.getVelocity().getValue();
    inputs.motorCurrent = motor.getStatorCurrent().getValue();
  }

  private void updateSimulation() {
    flywheelSim.update(Constants.SIM_TIME_PERIOD);

    TalonFXSimState simState = motor.getSimState();
    simState.setRotorAcceleration(flywheelSim.getAngularAcceleration());
    simState.setRotorVelocity(flywheelSim.getAngularVelocity());
    simState.setSupplyVoltage(Volts.of(12));
  }

  @Override
  public void close() {
    motor.close();
  }
}
