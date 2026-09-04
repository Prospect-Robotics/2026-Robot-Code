package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIOSim implements HopperIO {

  // Roller Motor simulation declaration.
  private final TalonFX feederMotor;
  private final TalonFXSimState feederMotorSimState;

  private final FlywheelSim feederSim; // Used for simulating voltage of the roller.

  public HopperIOSim() {
    feederMotor = new TalonFX(Constants.FEEDER_MOTOR_CAN_ID);
    feederMotor.getConfigurator().apply(HopperConstants.FEEDER_MOTOR_CONFIG);
    feederMotorSimState = feederMotor.getSimState();

    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                HopperConstants.FEEDER_SIM_MOI,
                HopperConstants.FEEDER_MOTOR_TO_ROLLER_GEARING),
            DCMotor.getKrakenX60(2));
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    updateSimulation();

    feederMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.feederMotorVoltage = feederMotor.getMotorVoltage().getValue();
    inputs.feederMotorRPS = feederMotor.getRotorVelocity().getValue();
    inputs.feederMotorStatorCurrent = feederMotor.getStatorCurrent().getValue();
    inputs.feederMotorSupplyCurrent = feederMotor.getSupplyCurrent().getValue();
  }

  public void updateSimulation() {
    feederSim.update(Constants.SIM_TIME_PERIOD);

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    feederMotorSimState.setRotorAcceleration(feederSim.getAngularAcceleration());
    feederMotorSimState.setRotorVelocity(feederSim.getAngularVelocity());
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    // Rollers
    feederMotor.setVoltage(rollerVoltage.in(Volts));
  }
}
