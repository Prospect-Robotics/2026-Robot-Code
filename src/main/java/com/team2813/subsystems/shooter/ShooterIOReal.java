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
  private TalonFX mainShooterMotor;
  private TalonFX followerShooterMotor;

  private AngularVelocity mainShooterSetpoint = RotationsPerSecond.of(0);

  public ShooterIOReal() {
    mainShooterMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR_ID);
    mainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);

    followerShooterMotor = new TalonFX(Constants.FOLLOWER_SHOOTER_MOTOR_ID);
    followerShooterMotor.getConfigurator().apply(ShooterConstants.FOLLOWER_SHOOTER_MOTOR_CONFIG);

    shooterVelocityControl = new VelocityVoltage(RotationsPerSecond.of(0));
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    inputs.mainShooterMotorVoltageVolts = mainShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.mainShooterMotorAngleRotations = mainShooterMotor.getPosition().getValue().in(Rotations);
    inputs.mainShooterMotorRotPerSec =
        mainShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.mainShooterMotorStatorCurrentAmps =
        mainShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.mainShooterMotorSupplyCurrentAmps =
        mainShooterMotor.getSupplyCurrent().getValue().in(Amps);
    inputs.mainShooterSetpointRotsPerSec = mainShooterSetpoint.in(RotationsPerSecond);

    inputs.followerShooterMotorVoltageVolts =
        followerShooterMotor.getMotorVoltage().getValue().in(Volts);
    inputs.followerShooterMotorRotPerSec =
        followerShooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.followerShooterMotorStatorCurrentAmps =
        followerShooterMotor.getStatorCurrent().getValue().in(Amps);
    inputs.followerShooterMotorSupplyCurrentAmps =
        followerShooterMotor.getSupplyCurrent().getValue().in(Amps);
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    // Uses Rot/s rather than passing AngularVelocity because there seems to be some issue with
    // AngularVelocity converting its value (i.e. Rot/s) to the base unit (rad/s)
    mainShooterSetpoint = shooterMotorVelocity;
    mainShooterMotor.setControl(shooterVelocityControl.withVelocity(shooterMotorVelocity));
    // same velocity
    followerShooterMotor.setControl(shooterVelocityControl);
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterVoltage) {
    mainShooterSetpoint = RotationsPerSecond.of(0);
    mainShooterMotor.setVoltage(shooterVoltage.in(Volts));
    followerShooterMotor.setVoltage(shooterVoltage.in(Volts));
  }
}
