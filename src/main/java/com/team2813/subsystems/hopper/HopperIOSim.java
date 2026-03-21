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

  private final TalonFX followerFeederMotor;
  private final TalonFXSimState followerFeederMotorSimState;

  private final FlywheelSim feederSim; // Used for simulating voltage of the roller.

  // Feeder Motor simulation declaration
  // Right motor when seen from the back (shooter side).
  private final TalonFX indexerMotor;
  private final TalonFXSimState indexerMotorSimState;

  public HopperIOSim() {
    mainFeederMotor = new TalonFX(Constants.MAIN_FEEDER_MOTOR_CAN_ID);
    mainFeederMotor.getConfigurator().apply(HopperConstants.MAIN_ROLLER_MOTOR_CONFIG);
    mainFeederMotorSimState = mainFeederMotor.getSimState();

    followerFeederMotor = new TalonFX(Constants.FOLLOWER_FEEDER_MOTOR_CAN_ID);
    followerFeederMotor.getConfigurator().apply(HopperConstants.FOLLOWER_FEEDER_MOTOR_CONFIG);
    //    followerRollerMotor.setControl(
    //        new Follower(Constants.MAIN_ROLLER_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));
    followerFeederMotorSimState = followerFeederMotor.getSimState();

    // The "0.01" value is the moment of inertia, as the CAD is not complete, a more accurate value
    // is unavailable.
    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                HopperConstants.FEEDER_SIM_MOI,
                HopperConstants.FEEDER_MOTOR_TO_ROLLER_GEARING),
            DCMotor.getKrakenX60(2));

    indexerMotor = new TalonFX(Constants.INDEXER_MOTOR_ID);
    indexerMotor.getConfigurator().apply(HopperConstants.INDEXER_MOTOR_CONFIG);
    indexerMotorSimState = indexerMotor.getSimState();
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    updateSimulation();

    mainFeederMotorSimState.setSupplyVoltage(Volts.of(12));
    followerFeederMotorSimState.setSupplyVoltage(Volts.of(12));
    indexerMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.mainFeederMotorVoltage = mainFeederMotor.getMotorVoltage().getValue();
    inputs.mainFeederMotorRPS = mainFeederMotor.getRotorVelocity().getValue();
    inputs.mainFeederMotorStatorCurrent = mainFeederMotor.getStatorCurrent().getValue();

    inputs.followerFeederMotorVoltage = followerFeederMotor.getMotorVoltage().getValue();
    inputs.followerFeederMotorRPS = followerFeederMotor.getRotorVelocity().getValue();
    inputs.followerFeederMotorStatorCurrent = followerFeederMotor.getStatorCurrent().getValue();

    inputs.indexerMotorVoltage = indexerMotor.getMotorVoltage().getValue();
    inputs.indexerMotorRPS = indexerMotor.getRotorVelocity().getValue();
    inputs.indexerMotorStatorCurrent = indexerMotor.getStatorCurrent().getValue();
  }

  public void updateSimulation() {
    feederSim.update(Constants.SIM_TIME_PERIOD);

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    mainFeederMotorSimState.setRotorAcceleration(feederSim.getAngularAcceleration());
    mainFeederMotorSimState.setRotorVelocity(feederSim.getAngularVelocity());

    // The follower roller motor is aligned with the main motor, so it gets the same values.
    followerFeederMotorSimState.setRotorAcceleration(feederSim.getAngularAcceleration());
    followerFeederMotorSimState.setRotorVelocity(feederSim.getAngularVelocity());
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    // Rollers
    mainFeederMotor.setVoltage(rollerVoltage.in(Volts));
    // Don't set the voltage of the follower motor, as this will be done automatically by the
    // Follower command.
    feederSim.setInputVoltage(rollerVoltage.in(Volts));

    // Feeders
    indexerMotor.setVoltage(feederVoltage.in(Volts));
  }
}
