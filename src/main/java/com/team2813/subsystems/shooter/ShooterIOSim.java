package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private TalonFX mainShooterMotor;
  private TalonFXSimState mainShooterSimState;

  private TalonFX followerShooterMotor;
  private TalonFXSimState followerShooterSimState;

  private FlywheelSim shooterSim;

  public ShooterIOSim() {
    mainShooterMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR_ID);
    mainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);
    mainShooterSimState = mainShooterMotor.getSimState();

    followerShooterMotor = new TalonFX(Constants.FOLLOWER_SHOOTER_MOTOR_ID);
    followerShooterMotor.setControl(ShooterConstants.FOLLOWER_SHOOTER_CONTROL_MODE);
    followerShooterSimState = followerShooterMotor.getSimState();

    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                ShooterConstants.SHOOTER_SIM_MOI, // "Moment of Inertia" taken from OnShape.
                ShooterConstants.SHOOTER_MOTOR_TO_FLYWHEEL_GEARING),
            DCMotor.getKrakenX60(2));
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    updateSimulation();

    mainShooterSimState.setSupplyVoltage(Volts.of(12));
    followerShooterSimState.setSupplyVoltage(Volts.of(12));
    kickerSimState.setSupplyVoltage(Volts.of(12));

    inputs.mainShooterMotorVoltage = mainShooterMotor.getMotorVoltage().getValue();
    inputs.mainShooterMotorRotPerSec = mainShooterMotor.getVelocity().getValue();
    inputs.mainShooterMotorCurrent = mainShooterMotor.getStatorCurrent().getValue();

    inputs.followerShooterMotorVoltage = followerShooterMotor.getMotorVoltage().getValue();
    inputs.followerShooterMotorRotPerSec = followerShooterMotor.getVelocity().getValue();
    inputs.followerShooterMotorCurrent = followerShooterMotor.getStatorCurrent().getValue();
  }

  public void updateSimulation() {
    // Update physics simulations every 20ms (like the actual bot).
    shooterSim.update(Constants.SIM_TIME_PERIOD);
    kickerSim.update(Constants.SIM_TIME_PERIOD);

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    mainShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration());
    mainShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity());
    // The follower roller motor is opposed with the main motor, so it gets negated values.
    followerShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration().unaryMinus());
    followerShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity().unaryMinus());
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterMotorVoltage) {
    mainShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    shooterSim.setInputVoltage(shooterMotorVoltage.in(Volts));
  }
}
