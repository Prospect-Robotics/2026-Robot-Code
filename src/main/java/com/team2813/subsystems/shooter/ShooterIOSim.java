package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final TalonFX mainShooterMotor;
  private final TalonFXSimState mainShooterSimState;

  private final TalonFX followerShooterMotor;
  private final TalonFXSimState followerShooterSimState;

  private final VelocityVoltage shooterVelocityControl;

  private final FlywheelSim shooterSim;

  private AngularVelocity mainShooterSetpoint = RotationsPerSecond.of(0);

  public ShooterIOSim() {
    mainShooterMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR_ID);
    mainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);
    mainShooterSimState = mainShooterMotor.getSimState();

    followerShooterMotor = new TalonFX(Constants.FOLLOWER_SHOOTER_MOTOR_ID);
    followerShooterMotor.setControl(ShooterConstants.FOLLOWER_SHOOTER_CONTROL_MODE);
    followerShooterSimState = followerShooterMotor.getSimState();

    shooterVelocityControl = new VelocityVoltage(RotationsPerSecond.of(0));

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

    inputs.mainShooterMotorVoltageVolts = mainShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.mainShooterMotorAngleRotations = mainShooterMotor.getPosition().getValue().in(Rotations);
    inputs.mainShooterMotorRotPerSec =
        mainShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.mainShooterMotorCurrentAmps = mainShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.mainShooterSetpointRotsPerSec = mainShooterSetpoint.in(RotationsPerSecond);

    inputs.followerShooterMotorVoltageVolts =
        followerShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.followerShooterMotorRotPerSec =
        followerShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.followerShooterMotorCurrentAmps =
        followerShooterMotor.getStatorCurrent().getValue().in(Amps);
  }

  public void updateSimulation() {
    // Update physics simulations every 20ms (like the actual bot).
    shooterSim.update(Constants.SIM_TIME_PERIOD);

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    mainShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration());
    mainShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity());
    // The follower roller motor is opposed with the main motor, so it gets negated values.
    followerShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration().unaryMinus());
    followerShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity().unaryMinus());
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    mainShooterSetpoint = shooterMotorVelocity;
    mainShooterMotor.setControl(
        shooterVelocityControl.withVelocity(shooterMotorVelocity.in(RotationsPerSecond)));

    shooterSim.setAngularVelocity(shooterMotorVelocity.in(RadiansPerSecond));
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterMotorVoltage) {
    mainShooterSetpoint = RotationsPerSecond.of(0);
    mainShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    shooterSim.setInputVoltage(shooterMotorVoltage.in(Volts));
  }
}
