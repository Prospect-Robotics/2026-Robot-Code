package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX mainShooterMotor;
  private final TalonFX followerShooterMotor;
  private final TalonFX kickerMotor;

  // Declaring the control here saves on having to create a new object each time.
  private final VelocityVoltage shooterVelocityControl;

  public ShooterIOReal() {
    mainShooterMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR_ID);
    mainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);

    followerShooterMotor = new TalonFX(Constants.FOLLOWER_SHOOTER_MOTOR_ID);
    followerShooterMotor.setControl(ShooterConstants.FOLLOWER_SHOOTER_CONTROL_MODE);

    kickerMotor = new TalonFX(Constants.KICKER_MOTOR_ID);
    kickerMotor.getConfigurator().apply(ShooterConstants.KICKER_MOTOR_CONFIG);

    shooterVelocityControl = new VelocityVoltage(RotationsPerSecond.of(0));
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    inputs.mainShooterMotorVoltage = mainShooterMotor.getMotorVoltage().getValue();
    inputs.mainShooterMotorRotPerSec = mainShooterMotor.getVelocity().getValue();
    inputs.mainShooterMotorCurrent = mainShooterMotor.getStatorCurrent().getValue();

    inputs.followerShooterMotorVoltage = followerShooterMotor.getMotorVoltage().getValue();
    inputs.followerShooterMotorRotPerSec = followerShooterMotor.getVelocity().getValue();
    inputs.followerShooterMotorCurrent = followerShooterMotor.getStatorCurrent().getValue();

    inputs.kickerMotorVoltage = kickerMotor.getMotorVoltage().getValue();
    inputs.kickerMotorRotPerSec = kickerMotor.getVelocity().getValue();
    inputs.kickerMotorCurrent = kickerMotor.getStatorCurrent().getValue();
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    mainShooterMotor.setControl(shooterVelocityControl.withVelocity(shooterMotorVelocity));
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterVoltage) {
    mainShooterMotor.setVoltage(shooterVoltage.in(Volts));
  }

  @Override
  public void setKickerMotorVoltage(Voltage kickerMotorVoltage) {
    kickerMotor.setVoltage(kickerMotorVoltage.in(Volts));
  }
}
