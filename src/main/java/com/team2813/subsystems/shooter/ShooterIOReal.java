package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {

  // Declaring the control here saves on having to create a new object each time.
  private final VelocityVoltage shooterVelocityControl;
  private TalonFX rightMainShooterMotor;
  private TalonFX leftFollowerShooterMotor;

  private AngularVelocity rightMainShooterSetpoint = RotationsPerSecond.of(0);

  public ShooterIOReal() {
    rightMainShooterMotor = new TalonFX(Constants.RIGHT_MAIN_SHOOTER_MOTOR_ID);
    rightMainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);

    leftFollowerShooterMotor = new TalonFX(Constants.LEFT_FOLLOWER_SHOOTER_MOTOR_ID);
    leftFollowerShooterMotor
        .getConfigurator()
        .apply(ShooterConstants.FOLLOWER_SHOOTER_MOTOR_CONFIG);

    shooterVelocityControl = new VelocityVoltage(RotationsPerSecond.of(0));
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    inputs.rightMainShooterMotorVoltageVolts =
        rightMainShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.rightMainShooterMotorAngleRotations =
        rightMainShooterMotor.getPosition().getValue().in(Rotations);
    inputs.rightMainShooterMotorRotPerSec =
        rightMainShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.rightMainShooterMotorStatorCurrentAmps =
        rightMainShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.rightMainShooterMotorSupplyCurrentAmps =
        rightMainShooterMotor.getSupplyCurrent().getValue().in(Amps);
    inputs.rightMainShooterSetpointRotsPerSec = rightMainShooterSetpoint.in(RotationsPerSecond);

    inputs.leftFollowerShooterMotorVoltageVolts =
        leftFollowerShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.leftFollowerShooterMotorRotPerSec =
        leftFollowerShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.leftFollowerShooterMotorStatorCurrentAmps =
        leftFollowerShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.leftFollowerShooterMotorSupplyCurrentAmps =
        leftFollowerShooterMotor.getSupplyCurrent().getValue().in(Amps);
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    // Uses Rot/s rather than passing AngularVelocity because there seems to be some issue with
    // AngularVelocity converting its value (i.e. Rot/s) to the base unit (rad/s)
    rightMainShooterSetpoint = shooterMotorVelocity;
    rightMainShooterMotor.setControl(shooterVelocityControl.withVelocity(shooterMotorVelocity));
    // same velocity
    leftFollowerShooterMotor.setControl(shooterVelocityControl);
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterVoltage) {
    rightMainShooterSetpoint = RotationsPerSecond.of(0);
    rightMainShooterMotor.setVoltage(shooterVoltage.in(Volts));
    leftFollowerShooterMotor.setVoltage(shooterVoltage.in(Volts));
  }
}
