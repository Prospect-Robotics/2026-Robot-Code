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
  private final TalonFX mainRollerMotor;
  private final TalonFXSimState mainRollerMotorSimState;

  private final TalonFX followerRollerMotor;
  private final TalonFXSimState followerRollerMotorSimState;

  private final FlywheelSim rollerSim; // Used for simulating voltage of the roller.

  // Feeder Motor simulation declaration
  // Right motor when seen from the back (shooter side).
  private final TalonFX feederMotor;
  private final TalonFXSimState feederMotorSimState;

  public HopperIOSim() {
    mainRollerMotor = new TalonFX(Constants.MAIN_ROLLER_MOTOR_CAN_ID);
    mainRollerMotor.getConfigurator().apply(HopperConstants.MAIN_ROLLER_MOTOR_CONFIG);
    mainRollerMotorSimState = mainRollerMotor.getSimState();

    followerRollerMotor = new TalonFX(Constants.FOLLOWER_ROLLER_MOTOR_CAN_ID);
    followerRollerMotor.getConfigurator().apply(HopperConstants.FOLLOWER_FEEDER_MOTOR_CONFIG);
    //    followerRollerMotor.setControl(
    //        new Follower(Constants.MAIN_ROLLER_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));
    followerRollerMotorSimState = followerRollerMotor.getSimState();

    // The "0.01" value is the moment of inertia, as the CAD is not complete, a more accurate value
    // is unavailable.
    rollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                HopperConstants.ROLLER_SIM_MOI,
                HopperConstants.ROLLER_MOTOR_TO_ROLLER_GEARING),
            DCMotor.getKrakenX60(2));

    feederMotor = new TalonFX(Constants.FEEDER_MOTOR_ID);
    feederMotor.getConfigurator().apply(HopperConstants.INDEXER_MOTOR_CONFIG);
    feederMotorSimState = feederMotor.getSimState();
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    updateSimulation();

    mainRollerMotorSimState.setSupplyVoltage(Volts.of(12));
    followerRollerMotorSimState.setSupplyVoltage(Volts.of(12));
    feederMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.mainRollerMotorVoltage = mainRollerMotor.getMotorVoltage().getValue();
    inputs.mainRollerMotorRPS = mainRollerMotor.getRotorVelocity().getValue();
    inputs.mainRollerMotorStatorCurrent = mainRollerMotor.getStatorCurrent().getValue();
    inputs.mainRollerMotorSupplyCurrent = mainRollerMotor.getSupplyCurrent().getValue();

    inputs.followerRollerMotorVoltage = followerRollerMotor.getMotorVoltage().getValue();
    inputs.followerRollerMotorRPS = followerRollerMotor.getRotorVelocity().getValue();
    inputs.followerRollerMotorStatorCurrent = followerRollerMotor.getStatorCurrent().getValue();
    inputs.followerRollerMotorSupplyCurrent = followerRollerMotor.getSupplyCurrent().getValue();

    inputs.feederVoltage = feederMotor.getMotorVoltage().getValue();
    inputs.feederRPS = feederMotor.getRotorVelocity().getValue();
    inputs.feederStatorCurrent = feederMotor.getStatorCurrent().getValue();
    inputs.feederSupplyCurrent = feederMotor.getSupplyCurrent().getValue();
  }

  public void updateSimulation() {
    rollerSim.update(Constants.SIM_TIME_PERIOD);

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    mainRollerMotorSimState.setRotorAcceleration(rollerSim.getAngularAcceleration());
    mainRollerMotorSimState.setRotorVelocity(rollerSim.getAngularVelocity());

    // The follower roller motor is aligned with the main motor, so it gets the same values.
    followerRollerMotorSimState.setRotorAcceleration(rollerSim.getAngularAcceleration());
    followerRollerMotorSimState.setRotorVelocity(rollerSim.getAngularVelocity());
  }

  @Override
  public void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {
    // Rollers
    mainRollerMotor.setVoltage(rollerVoltage.in(Volts));
    // Don't set the voltage of the follower motor, as this will be done automatically by the
    // Follower command.
    rollerSim.setInputVoltage(rollerVoltage.in(Volts));

    // Feeders
    feederMotor.setVoltage(feederVoltage.in(Volts));
  }
}
