package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged replayedInputs;

  public Hopper(HopperIO io) {
    this.io = io;
    this.replayedInputs = new HopperIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    Logger.processInputs("Hopper", replayedInputs);
  }

  public void intake() {
    io.setMotorVoltage(
        HopperConstants.getRollerIntakeVoltage(),
        HopperConstants.getRightFeederIntakeVoltage(),
        HopperConstants.getLeftFeederIntakeVoltage());
  }

  public void outtake() {
    io.setMotorVoltage(
        HopperConstants.getRollerOuttakeVoltage(),
        HopperConstants.getRightFeederOuttakeVoltage(),
        HopperConstants.getLeftFeederOuttakeVoltage());
  }

  public void stop() {
    io.setMotorVoltage(Volts.of(0), Volts.of(0), Volts.of(0));
  }

  public Command intakeCommand() {
    return new InstantCommand(this::intake, this);
  }

  public Command outtakeCommand() {
    return new InstantCommand(this::outtake, this);
  }

  public Command stopCommand() {
    return new InstantCommand(this::stop, this);
  }
}
