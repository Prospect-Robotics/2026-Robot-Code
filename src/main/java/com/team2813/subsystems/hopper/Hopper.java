package com.team2813.subsystems.hopper;

import static com.team2813.subsystems.hopper.HopperConstants.*;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged replayedInputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    Logger.processInputs("Hopper", replayedInputs);
  }

  public void intake() {
    io.setMotorVoltage(
        HopperConstants.ROLLER_INTAKE_VOLTAGE, HopperConstants.FEEDER_INTAKE_VOLTAGE);
  }

  public void outtake() {
    io.setMotorVoltage(
        HopperConstants.ROLLER_OUTTAKE_VOLTAGE, HopperConstants.FEEDER_OUTTAKE_VOLTAGE);
  }

  public void stop() {
    io.setMotorVoltage(Volts.of(0), Volts.of(0));
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> intake());
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> outtake());
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stop());
  }
}
