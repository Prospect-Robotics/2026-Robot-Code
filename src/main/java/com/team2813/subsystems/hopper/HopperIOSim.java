package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
  private final TalonFX rightFeederMotor;
  private final TalonFXSimState rightFeederMotorSimState;

  private FlywheelSim rightFeederModuleSim;

  // Left motor when seen from the back (shooter side).
  private final TalonFX leftFeederMotor;
  private final TalonFXSimState leftFeederMotorSimState;

  private FlywheelSim leftFeederModuleSim;

  public HopperIOSim() {
    mainRollerMotor = new TalonFX(Constants.MAIN_ROLLER_MOTOR_CAN_ID);
    mainRollerMotor.getConfigurator().apply(HopperConstants.ROLLER_MOTOR_CONFIG);
    mainRollerMotorSimState = mainRollerMotor.getSimState();

    followerRollerMotor = new TalonFX(Constants.FOLLOWER_ROLLER_MOTOR_CAN_ID);
    followerRollerMotor.setControl(
        new Follower(Constants.MAIN_ROLLER_MOTOR_CAN_ID, MotorAlignmentValue.Aligned));
    followerRollerMotorSimState = followerRollerMotor.getSimState();

    // The "0.01" value is the moment of inertia, as the CAD is not complete, a more accurate value
    // is unavailable.
    rollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), 0.01, HopperConstants.ROLLER_MOTOR_TO_ROLLER_GEARING),
            DCMotor.getKrakenX60(2));

    rightFeederMotor = new TalonFX(Constants.RIGHT_FEEDER_MOTOR_ID);
    rightFeederMotor.getConfigurator().apply(HopperConstants.RIGHT_FEEDER_MOTOR_CONFIG);
    rightFeederMotorSimState = rightFeederMotor.getSimState();

    leftFeederMotor = new TalonFX(Constants.LEFT_FEEDER_MOTOR_ID);
    leftFeederMotor.getConfigurator().apply(HopperConstants.LEFT_FEEDER_MOTOR_CONFIG);
    leftFeederMotorSimState = leftFeederMotor.getSimState();
  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    updateSimulation();

    mainRollerMotorSimState.setSupplyVoltage(Volts.of(12));
    followerRollerMotorSimState.setSupplyVoltage(Volts.of(12));
    rightFeederMotorSimState.setSupplyVoltage(Volts.of(12));
    leftFeederMotorSimState.setSupplyVoltage(Volts.of(12));

    inputs.mainRollerMotorVoltage = mainRollerMotor.getMotorVoltage().getValue();
    inputs.mainRollerMotorRPS = mainRollerMotor.getRotorVelocity().getValue();
    inputs.mainRollerMotorCurrent = mainRollerMotor.getStatorCurrent().getValue();

    inputs.followerRollerMotorVoltage = followerRollerMotor.getMotorVoltage().getValue();
    inputs.followerRollerMotorRPS = followerRollerMotor.getRotorVelocity().getValue();
    inputs.followerRollerMotorCurrent = followerRollerMotor.getStatorCurrent().getValue();

    inputs.rightFeederVoltage = rightFeederMotor.getMotorVoltage().getValue();
    inputs.rightFeederRPS = rightFeederMotor.getRotorVelocity().getValue();
    inputs.rightFeederCurrent = rightFeederMotor.getStatorCurrent().getValue();

    inputs.leftFeederVoltage = leftFeederMotor.getMotorVoltage().getValue();
    inputs.leftFeederRPS = leftFeederMotor.getRotorVelocity().getValue();
    inputs.leftFeederCurrent = leftFeederMotor.getStatorCurrent().getValue();
  }

  public void updateSimulation() {
    // Update physics simulations every 20ms (like the actual bot).
    rollerSim.update(0.02);

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    mainRollerMotorSimState.setRotorAcceleration(rollerSim.getAngularAcceleration());
    mainRollerMotorSimState.setRotorVelocity(rollerSim.getAngularVelocity());

    // The follower roller motor is aligned with the main motor, so it gets the same values.
    followerRollerMotorSimState.setRotorAcceleration(rollerSim.getAngularAcceleration());
    followerRollerMotorSimState.setRotorVelocity(rollerSim.getAngularVelocity());
  }

  @Override
  public void setMotorVoltage(
      Voltage rollerVoltage, Voltage rightFeederVoltage, Voltage leftFeederVoltage) {
    // Rollers
    mainRollerMotor.setVoltage(rollerVoltage.in(Volts));
    // Don't set the voltage of the follower motor, as this will be done automatically by the
    // Follower command.
    rollerSim.setInputVoltage(rollerVoltage.in(Volts));

    // Feeders
    rightFeederMotor.setVoltage(rightFeederVoltage.in(Volts));
    leftFeederMotor.setVoltage(leftFeederVoltage.in(Volts));
  }
}
