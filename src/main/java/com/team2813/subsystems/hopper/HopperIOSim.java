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
  private final TalonFX mainFeederMotor;
  private final TalonFXSimState mainFeederMotorSimState;

  private final FlywheelSim feederSim; // Used for simulating voltage of the roller.

  public HopperIOSim() {
    mainFeederMotor = new TalonFX(Constants.MAIN_FEEDER_MOTOR_CAN_ID);
    mainFeederMotor.getConfigurator().apply(HopperConstants.MAIN_ROLLER_MOTOR_CONFIG);
    mainFeederMotorSimState = mainFeederMotor.getSimState();

    // The "0.01" value is the moment of inertia, as the CAD is not complete, a more accurate value
    // is unavailable.
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

    mainFeederMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.mainFeederMotorVoltage = mainFeederMotor.getMotorVoltage().getValue();
    inputs.mainFeederMotorRPS = mainFeederMotor.getRotorVelocity().getValue();
    inputs.mainFeederMotorStatorCurrent = mainFeederMotor.getStatorCurrent().getValue();
    inputs.mainFeederMotorSupplyCurrent = mainFeederMotor.getSupplyCurrent().getValue();
  }

  public void updateSimulation() {
    feederSim.update(Constants.SIM_TIME_PERIOD);

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    mainFeederMotorSimState.setRotorAcceleration(feederSim.getAngularAcceleration());
    mainFeederMotorSimState.setRotorVelocity(feederSim.getAngularVelocity());
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    // Rollers
    mainFeederMotor.setVoltage(rollerVoltage.in(Volts));
  }
}
