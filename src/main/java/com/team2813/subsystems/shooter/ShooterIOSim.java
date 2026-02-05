package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOSim implements ShooterIO {
  private TalonFX mainShooterMotor;
  private TalonFXSimState mainShooterSimState;

  private TalonFX followerShooterMotor;
  private TalonFXSimState followerShooterSimState;

  private TalonFX kickerMotor;
  private TalonFXSimState kickerSimState;

  public ShooterIOSim() {
    mainShooterMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR_ID);
    mainShooterMotor.getConfigurator().apply(ShooterConstants.MAIN_SHOOTER_MOTOR_CONFIG);
    mainShooterSimState = mainShooterMotor.getSimState();

    followerShooterMotor = new TalonFX(Constants.FOLLOWER_SHOOTER_MOTOR_ID);
    followerShooterMotor.setControl(ShooterConstants.FOLLOWER_SHOOTER_CONTROL_MODE);
    followerShooterSimState = followerShooterMotor.getSimState();

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

  public void updateSimulation() {}

  @Override
  public void setShooterMotorVoltage(Voltage shooterMotorVoltage) {
    mainShooterMotor.setVoltage(shooterMotorVoltage.in(Volts));
  }

  @Override
  public void setKickerMotorVoltage(Voltage kickerMotorVoltage) {
    kickerMotor.setVoltage(kickerMotorVoltage.in(Volts));
  }
}
