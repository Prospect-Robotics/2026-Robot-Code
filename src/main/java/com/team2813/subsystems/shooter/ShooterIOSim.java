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
  private final TalonFX upperRightShooterMotor;
  private final TalonFXSimState upperRightShooterSimState;

  private final TalonFX lowerRightShooterMotor;
  private final TalonFXSimState lowerRightShooterSimState;

  private final TalonFX upperLeftShooterMotor;
  private final TalonFXSimState upperLeftShooterSimState;

  private final TalonFX lowerLeftShooterMotor;
  private final TalonFXSimState lowerLeftShooterSimState;

  private final VelocityVoltage shooterVelocityControl;

  private final FlywheelSim shooterSim;

  private AngularVelocity upperRightShooterSetpoint = RotationsPerSecond.of(0);

  public ShooterIOSim() {
    upperRightShooterMotor = new TalonFX(Constants.UPPER_RIGHT_SHOOTER_MOTOR_ID);
    upperRightShooterMotor
        .getConfigurator()
        .apply(ShooterConstants.UPPER_RIGHT_SHOOTER_MOTOR_CONFIG);
    upperRightShooterSimState = upperRightShooterMotor.getSimState();

    lowerRightShooterMotor = new TalonFX(Constants.LOWER_RIGHT_SHOOTER_MOTOR_ID);
    lowerRightShooterMotor
        .getConfigurator()
        .apply(ShooterConstants.LOWER_RIGHT_SHOOTER_MOTOR_CONFIG);
    lowerRightShooterSimState = lowerRightShooterMotor.getSimState();

    upperLeftShooterMotor = new TalonFX(Constants.UPPER_LEFT_SHOOTER_MOTOR_ID);
    upperLeftShooterMotor.getConfigurator().apply(ShooterConstants.UPPER_LEFT_SHOOTER_MOTOR_CONFIG);
    upperLeftShooterSimState = upperLeftShooterMotor.getSimState();

    lowerLeftShooterMotor = new TalonFX(Constants.LOWER_LEFT_SHOOTER_MOTOR_ID);
    lowerLeftShooterMotor.getConfigurator().apply(ShooterConstants.LOWER_LEFT_SHOOTER_MOTOR_CONFIG);
    lowerLeftShooterSimState = lowerLeftShooterMotor.getSimState();

    shooterVelocityControl = new VelocityVoltage(RotationsPerSecond.of(0));

    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(4),
                ShooterConstants.SHOOTER_SIM_MOI, // "Moment of Inertia" taken from OnShape.
                ShooterConstants.SHOOTER_MOTOR_TO_FLYWHEEL_GEARING),
            DCMotor.getKrakenX60(4));
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    updateSimulation();
    inputs.upperRightShooterSetpointRotsPerSec = upperRightShooterSetpoint.in(RotationsPerSecond);

    inputs.upperRightShooterMotorVoltageVolts =
        upperRightShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.upperRightShooterMotorRotPerSec =
        upperRightShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.upperRightShooterMotorStatorCurrentAmps =
        upperRightShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.upperRightShooterMotorSupplyCurrentAmps =
        upperRightShooterMotor.getSupplyCurrent().getValue().in(Amps);

    inputs.upperRightShooterMotorAngleRotations =
        upperRightShooterMotor.getPosition().getValue().in(Rotations);

    inputs.lowerRightShooterMotorVoltageVolts =
        lowerRightShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.lowerRightShooterMotorRotPerSec =
        lowerRightShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.lowerRightShooterMotorStatorCurrentAmps =
        lowerRightShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.lowerRightShooterMotorSupplyCurrentAmps =
        lowerRightShooterMotor.getSupplyCurrent().getValue().in(Amps);

    inputs.upperLeftShooterMotorVoltageVolts =
        upperLeftShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.upperLeftShooterMotorRotPerSec =
        upperLeftShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.upperLeftShooterMotorStatorCurrentAmps =
        upperLeftShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.upperLeftShooterMotorSupplyCurrentAmps =
        upperLeftShooterMotor.getSupplyCurrent().getValue().in(Amps);

    inputs.upperLeftShooterMotorAngleRotations =
        upperLeftShooterMotor.getPosition().getValue().in(Rotations);

    inputs.lowerLeftShooterMotorVoltageVolts =
        lowerLeftShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.lowerLeftShooterMotorRotPerSec =
        lowerLeftShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.lowerLeftShooterMotorStatorCurrentAmps =
        lowerLeftShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.lowerLeftShooterMotorSupplyCurrentAmps =
        lowerLeftShooterMotor.getSupplyCurrent().getValue().in(Amps);
  }

  public void updateSimulation() {
    // Update physics simulations every 20ms (like the actual bot).
    shooterSim.update(Constants.SIM_TIME_PERIOD);

    upperRightShooterSimState.setSupplyVoltage(Volts.of(12));
    lowerRightShooterSimState.setSupplyVoltage(Volts.of(12));
    upperLeftShooterSimState.setSupplyVoltage(Volts.of(12));
    lowerLeftShooterSimState.setSupplyVoltage(Volts.of(12));

    // Feed the velocity and acceleration of the roller simulation into the simulation motors to
    // accurately model them.
    upperRightShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration());
    upperRightShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity());

    lowerLeftShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration());
    lowerLeftShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity());

    // The follower roller motor is opposed with the main motor, so it gets negated values.
    upperLeftShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration().unaryMinus());
    upperLeftShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity().unaryMinus());

    lowerLeftShooterSimState.setRotorAcceleration(shooterSim.getAngularAcceleration().unaryMinus());
    lowerLeftShooterSimState.setRotorVelocity(shooterSim.getAngularVelocity().unaryMinus());
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    upperRightShooterSetpoint = shooterMotorVelocity;
    upperRightShooterMotor.setControl(
        shooterVelocityControl.withVelocity(shooterMotorVelocity.in(RotationsPerSecond)));
    lowerRightShooterMotor.setControl(shooterVelocityControl);
    upperLeftShooterMotor.setControl(shooterVelocityControl);
    lowerLeftShooterMotor.setControl(shooterVelocityControl);

    shooterSim.setAngularVelocity(shooterMotorVelocity.in(RadiansPerSecond));
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterMotorVoltage) {
    upperRightShooterSetpoint = RotationsPerSecond.of(0);
    upperRightShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    lowerRightShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    upperLeftShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    lowerRightShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
    shooterSim.setInputVoltage(shooterMotorVoltage.in(Volts));
  }
}
