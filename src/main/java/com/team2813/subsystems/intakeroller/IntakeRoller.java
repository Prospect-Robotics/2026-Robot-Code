package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged replayedInputs;

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
    replayedInputs = new IntakeRollerIOInputsAutoLogged();
  }

  public void intake() {
    io.setIntakeMotorVoltage(IntakeRollerConstants.getIntakeVoltage());
  }

  public void outtake() {
    io.setIntakeMotorVoltage(IntakeRollerConstants.getOuttakeVoltage());
  }

  public void stop() {
    io.setIntakeMotorVoltage(Volts.of(0));
  }

  public Command intakeCommand() {
    return new RunCommand(this::intake).andThen(this::stop);
  }

  public Command outtakeCommand() {
    return new RunCommand(this::outtake).andThen(this::stop);
  }
}
