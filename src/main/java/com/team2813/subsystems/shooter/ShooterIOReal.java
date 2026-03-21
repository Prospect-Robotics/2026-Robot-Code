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
  private TalonFX shooterMotor1;
  private TalonFX shooterMotor2;
  private TalonFX shooterMotor3;
  private TalonFX shooterMotor4;

  private AngularVelocity mainShooterSetpoint = RotationsPerSecond.of(0);

  public ShooterIOReal() {
    shooterMotor1 = new TalonFX(Constants.SHOOTER_MOTOR_1_ID);
    shooterMotor1.getConfigurator().apply(ShooterConstants.SHOOTER_MOTOR_1_CONFIG);

    shooterMotor2 = new TalonFX(Constants.SHOOTER_MOTOR_2_ID);
    shooterMotor2.getConfigurator().apply(ShooterConstants.SHOOTER_MOTOR_2_CONFIG);

    shooterMotor3 = new TalonFX(Constants.SHOOTER_MOTOR_3_ID);
    shooterMotor3.getConfigurator().apply(ShooterConstants.SHOOTER_MOTOR_3_CONFIG);

    shooterMotor4 = new TalonFX(Constants.SHOOTER_MOTOR_4_ID);
    shooterMotor4.getConfigurator().apply(ShooterConstants.SHOOTER_MOTOR_4_CONFIG);

    shooterVelocityControl = new VelocityVoltage(RotationsPerSecond.of(0));
  }

  @Override
  public void updateState(ShooterIOInputs inputs) {
    inputs.shooterMotor1VoltageVolts = shooterMotor1.getMotorVoltage().getValue().in(Volts);
    inputs.shooterMotor1AngleRotations = shooterMotor1.getPosition().getValue().in(Rotations);
    inputs.shooterMotor1RotPerSec = shooterMotor1.getVelocity().getValue().in(RotationsPerSecond);
    inputs.shooterMotor1CurrentAmps = shooterMotor1.getStatorCurrent().getValue().in(Amps);
    inputs.shooter1SetpointRotsPerSec = mainShooterSetpoint.in(RotationsPerSecond);

    inputs.shooterMotor2VoltageVolts = shooterMotor2.getMotorVoltage().getValue().in(Volts);
    inputs.shooterMotor2AngleRotations = shooterMotor2.getPosition().getValue().in(Rotations);
    inputs.shooterMotor2RotPerSec = shooterMotor2.getVelocity().getValue().in(RotationsPerSecond);
    inputs.shooterMotor2CurrentAmps = shooterMotor2.getStatorCurrent().getValue().in(Amps);
    inputs.shooter2SetpointRotsPerSec = mainShooterSetpoint.in(RotationsPerSecond);

    inputs.shooterMotor3VoltageVolts = shooterMotor3.getMotorVoltage().getValue().in(Volts);
    inputs.shooterMotor3AngleRotations = shooterMotor3.getPosition().getValue().in(Rotations);
    inputs.shooterMotor3RotPerSec = shooterMotor3.getVelocity().getValue().in(RotationsPerSecond);
    inputs.shooterMotor3CurrentAmps = shooterMotor3.getStatorCurrent().getValue().in(Amps);
    inputs.shooter3SetpointRotsPerSec = mainShooterSetpoint.in(RotationsPerSecond);

    inputs.shooterMotor4VoltageVolts = shooterMotor4.getMotorVoltage().getValue().in(Volts);
    inputs.shooterMotor4AngleRotations = shooterMotor4.getPosition().getValue().in(Rotations);
    inputs.shooterMotor4RotPerSec = shooterMotor4.getVelocity().getValue().in(RotationsPerSecond);
    inputs.shooterMotor4CurrentAmps = shooterMotor4.getStatorCurrent().getValue().in(Amps);
    inputs.shooter4SetpointRotsPerSec = mainShooterSetpoint.in(RotationsPerSecond);
  }

  @Override
  public void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {
    // Uses Rot/s rather than passing AngularVelocity because there seems to be some issue with
    // AngularVelocity converting its value (i.e. Rot/s) to the base unit (rad/s)
    mainShooterSetpoint = shooterMotorVelocity;
    shooterMotor1.setControl(shooterVelocityControl.withVelocity(shooterMotorVelocity));
    // same velocity
    shooterMotor2.setControl(shooterVelocityControl);

    shooterMotor3.setControl(shooterVelocityControl);
    shooterMotor4.setControl(shooterVelocityControl);
  }

  @Override
  public void setShooterMotorVoltage(Voltage shooterVoltage) {
    mainShooterSetpoint = RotationsPerSecond.of(0);
    shooterMotor1.setVoltage(shooterVoltage.in(Volts));
    shooterMotor2.setVoltage(shooterVoltage.in(Volts));
    shooterMotor3.setVoltage(shooterVoltage.in(Volts));
    shooterMotor4.setVoltage(shooterVoltage.in(Volts));
  }
}
