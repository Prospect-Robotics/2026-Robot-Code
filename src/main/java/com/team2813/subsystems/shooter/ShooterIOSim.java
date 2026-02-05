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

  private TalonFX kickerMotor;
  private TalonFXSimState kickerSimState;

  public ShooterIOSim() {
    mainShooterMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR_ID);
    mainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);
    mainShooterSimState = mainShooterMotor.getSimState();

    followerShooterMotor = new TalonFX(Constants.FOLLOWER_SHOOTER_MOTOR_ID);
    followerShooterMotor.setControl(ShooterConstants.FOLLOWER_SHOOTER_CONTROL_MODE);
    followerShooterSimState = followerShooterMotor.getSimState();
    // MOI taken from onshape.
    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                0.00303431,
                ShooterConstants.SHOOTER_MOTOR_TO_FLYWHEEL_GEARING),
            DCMotor.getKrakenX60(2));

    kickerMotor = new TalonFX(Constants.KICKER_MOTOR_ID);
    kickerMotor.getConfigurator().apply(ShooterConstants.KICKER_MOTOR_CONFIG);
    kickerSimState = kickerMotor.getSimState();
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    updateSimulation();

    mainShooterSimState.setSupplyVoltage(Volts.of(12));
    followerShooterSimState.setSupplyVoltage(Volts.of(12));
    kickerSimState.setSupplyVoltage(Volts.of(12));

    inputs.mainShooterMotorVoltage = mainShooterMotor.getMotorVoltage().getValue();
    inputs.mainShooterMotorRPS = mainShooterMotor.getVelocity().getValue();
    inputs.mainShooterMotorCurrent = mainShooterMotor.getStatorCurrent().getValue();

    inputs.followerShooterMotorVoltage = followerShooterMotor.getMotorVoltage().getValue();
    inputs.followerShooterMotorRPS = followerShooterMotor.getVelocity().getValue();
    inputs.followerShooterMotorCurrent = followerShooterMotor.getStatorCurrent().getValue();

    inputs.kickerMotorVoltage = kickerMotor.getMotorVoltage().getValue();
    inputs.kickerMotorRPS = kickerMotor.getVelocity().getValue();
    inputs.kickerMotorCurrent = kickerMotor.getStatorCurrent().getValue();
  }

  public void updateSimulation() {
    // Update physics simulations every 20ms (like the actual bot).
    shooterSim.update(0.02);

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
  }

  @Override
  public void setKickerMotorVoltage(Voltage kickerMotorVoltage) {
    kickerMotor.setVoltage(kickerMotorVoltage.in(Volts));
  }
}
