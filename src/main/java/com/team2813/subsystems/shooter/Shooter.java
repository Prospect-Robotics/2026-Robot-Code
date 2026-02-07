package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void intake() {
    io.setMotorVoltage(
        ShooterConstants.getShooterIntakeVoltage(), ShooterConstants.getKickerIntakeVoltage());
  }

  public void outtake() {
    io.setMotorVoltage(
        ShooterConstants.getShooterOuttakeVoltage(), ShooterConstants.getKickerOuttakeVoltage());
  }

  public void stop() {
    io.setMotorVoltage(Volts.of(0), Volts.of(0));
  }

  public Command intakeCommand() {
    return new InstantCommand(this::intake, this);
  }

  public Command outakeCommand() {
    return new InstantCommand(this::outtake, this);
  }

  public Command stopCommand() {
    return new InstantCommand(this::stop, this);
  }

  // Used for auto calculated motor speed.
  public void setShooterMotorVoltage(Voltage voltage) {
    io.setShooterMotorVoltage(voltage);
  }

}
