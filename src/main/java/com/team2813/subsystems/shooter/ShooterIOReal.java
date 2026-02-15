package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
  private TalonFX mainShooterMotor;
  private TalonFX followerShooterMotor;
  private TalonFX kickerMotor;

  public ShooterIOReal() {
    mainShooterMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR_ID);
    mainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);

    followerShooterMotor = new TalonFX(Constants.FOLLOWER_SHOOTER_MOTOR_ID);
    followerShooterMotor.setControl(ShooterConstants.FOLLOWER_SHOOTER_CONTROL_MODE);

    kickerMotor = new TalonFX(Constants.KICKER_MOTOR_ID);
    kickerMotor.getConfigurator().apply(ShooterConstants.KICKER_MOTOR_CONFIG);
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
  public void setShooterMotorVoltage(Voltage shooterVoltage) {
    mainShooterMotor.setVoltage(shooterVoltage.in(Volts));
  }

  @Override
  public void setKickerMotorVoltage(Voltage kickerMotorVoltage) {
    kickerMotor.setVoltage(kickerMotorVoltage.in(Volts));
  }
}
