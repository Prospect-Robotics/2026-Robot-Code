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
  private TalonFX upperRightShooterMotor;
  private TalonFX lowerRightShooterMotor;

  private TalonFX upperLeftShooterMotor;
  private TalonFX lowerLeftShooterMotor;

  private AngularVelocity upperRightShooterSetpoint = RotationsPerSecond.of(0);

  public ShooterIOReal() {
    upperRightShooterMotor = new TalonFX(Constants.UPPER_RIGHT_SHOOTER_MOTOR_ID);
    upperRightShooterMotor
        .getConfigurator()
        .apply(ShooterConstants.UPPER_RIGHT_SHOOTER_MOTOR_CONFIG);

    lowerRightShooterMotor = new TalonFX(Constants.LOWER_RIGHT_SHOOTER_MOTOR_ID);
    lowerRightShooterMotor
        .getConfigurator()
        .apply(ShooterConstants.LOWER_RIGHT_SHOOTER_MOTOR_CONFIG);

    upperLeftShooterMotor = new TalonFX(Constants.UPPER_LEFT_SHOOTER_MOTOR_ID);
    upperLeftShooterMotor.getConfigurator().apply(ShooterConstants.UPPER_LEFT_SHOOTER_MOTOR_CONFIG);

    lowerLeftShooterMotor = new TalonFX(Constants.LOWER_LEFT_SHOOTER_MOTOR_ID);
    lowerLeftShooterMotor.getConfigurator().apply(ShooterConstants.LOWER_LEFT_SHOOTER_MOTOR_CONFIG);

    shooterVelocityControl = new VelocityVoltage(RotationsPerSecond.of(0));
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    inputs.upperRightShooterSetpointRotsPerSec = upperRightShooterSetpoint.in(RotationsPerSecond);

    inputs.upperRightShooterMotorVoltageVolts =
        upperRightShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.upperRightShooterMotorAngleRotations =
        upperRightShooterMotor.getPosition().getValue().in(Rotations);
    inputs.upperRightShooterMotorRotPerSec =
        upperRightShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.upperRightShooterMotorStatorCurrentAmps =
        upperRightShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.upperRightShooterMotorSupplyCurrentAmps =
        upperRightShooterMotor.getSupplyCurrent().getValue().in(Amps);

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
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    // Uses Rot/s rather than passing AngularVelocity because there seems to be some issue with
    // AngularVelocity converting its value (i.e. Rot/s) to the base unit (rad/s)
    upperRightShooterSetpoint = shooterMotorVelocity;
    upperRightShooterMotor.setControl(shooterVelocityControl.withVelocity(shooterMotorVelocity));
    // same velocity
    upperLeftShooterMotor.setControl(shooterVelocityControl);
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterVoltage) {
    upperRightShooterSetpoint = RotationsPerSecond.of(0);
    upperRightShooterMotor.setVoltage(shooterVoltage.in(Volts));
    upperLeftShooterMotor.setVoltage(shooterVoltage.in(Volts));
  }
}
