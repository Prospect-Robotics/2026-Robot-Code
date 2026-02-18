package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged replayedInputs;

  public Shooter(ShooterIO io) {
    this.io = io;
    this.replayedInputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    Logger.processInputs("Shooter", replayedInputs);
  }

  public void stop() {
    io.setMotorVoltage(Volts.of(0), Volts.of(0));
  }

  // Waits before starting the kicker to allow the shooter flywheel to get up to speed.][\

  public Command intakeCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> io.setShooterMotorVoltage(ShooterConstants.getShooterIntakeVoltage())),
        new WaitCommand(Seconds.of(2)),
        new StartEndCommand(
            () -> io.setKickerMotorVoltage(ShooterConstants.getKickerIntakeVoltage()),
            this::stop,
            this));
  }

  public Command outakeCommand() {
    return new StartEndCommand(
        () ->
            io.setMotorVoltage(
                ShooterConstants.getShooterOuttakeVoltage(),
                ShooterConstants.getKickerOuttakeVoltage()),
        this::stop);
  }

  // Used for auto calculated motor speed.
  public void setShooterMotorVoltage(Voltage voltage) {
    io.setShooterMotorVoltage(voltage);
  }
}
