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
  private final TalonFX rightMainShooterMotor;
  private final TalonFXSimState rightMainShooterSimState;

  private final TalonFX leftFollowerShooterMotor;
  private final TalonFXSimState leftFollowerShooterSimState;

  private final VelocityVoltage shooterVelocityControl;

  private final FlywheelSim shooterSim;

  private AngularVelocity rightMainShooterSetpoint = RotationsPerSecond.of(0);

  public ShooterIOSim() {
    rightMainShooterMotor = new TalonFX(Constants.RIGHT_MAIN_SHOOTER_MOTOR_ID);
    rightMainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);
    rightMainShooterSimState = rightMainShooterMotor.getSimState();

    leftFollowerShooterMotor = new TalonFX(Constants.LEFT_FOLLOWER_SHOOTER_MOTOR_ID);
    leftFollowerShooterMotor
        .getConfigurator()
        .apply(ShooterConstants.FOLLOWER_SHOOTER_MOTOR_CONFIG);
    leftFollowerShooterSimState = leftFollowerShooterMotor.getSimState();

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

    inputs.rightMainShooterMotorVoltageVolts =
        rightMainShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.rightMainShooterMotorAngleRotations =
        rightMainShooterMotor.getPosition().getValue().in(Rotations);
    inputs.rightMainShooterMotorRotPerSec =
        rightMainShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.rightMainShooterMotorStatorCurrentAmps =
        rightMainShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.rightMainShooterSetpointRotsPerSec = rightMainShooterSetpoint.in(RotationsPerSecond);
    inputs.rightMainShooterMotorSupplyCurrentAmps =
        rightMainShooterMotor.getSupplyCurrent().getValue().in(Amps);

    inputs.leftFollowerShooterMotorVoltageVolts =
        leftFollowerShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.leftFollowerShooterMotorRotPerSec =
        leftFollowerShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.leftFollowerShooterMotorStatorCurrentAmps =
        leftFollowerShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.leftFollowerShooterMotorSupplyCurrentAmps =
        leftFollowerShooterMotor.getSupplyCurrent().getValue().in(Amps);
  }

  public void updateSimulation() {
    // Update physics simulations every 20ms (like the actual bot).
    shooterSim.update(Constants.SIM_TIME_PERIOD);

    rightMainShooterSimState.setSupplyVoltage(Volts.of(12));
    leftFollowerShooterSimState.setSupplyVoltage(Volts.of(12));

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    rightMainShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration());
    rightMainShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity());
    // The follower roller motor is opposed with the main motor, so it gets negated values.
    leftFollowerShooterSimState.setRotorAcceleration(
        shooterSim.getAngularAcceleration().unaryMinus());
    leftFollowerShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity().unaryMinus());
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    rightMainShooterSetpoint = shooterMotorVelocity;
    rightMainShooterMotor.setControl(
        shooterVelocityControl.withVelocity(shooterMotorVelocity.in(RotationsPerSecond)));
    leftFollowerShooterMotor.setControl(shooterVelocityControl);

    shooterSim.setAngularVelocity(shooterMotorVelocity.in(RadiansPerSecond));
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterMotorVoltage) {
    rightMainShooterSetpoint = RotationsPerSecond.of(0);
    rightMainShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    leftFollowerShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    shooterSim.setInputVoltage(shooterMotorVoltage.in(Volts));
  }
}
