package com.team2813.subsystems.intake;

import static com.team2813.subsystems.intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged replayedInput = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void intake() {
    io.setMotorVoltage(INTAKE_VOLTAGE);
  }

  public void outtake() {
    io.setMotorVoltage(OUTTAKE_VOLTAGE);
  }

  public void stop() {
    io.setMotorVoltage(Volts.of(0));
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
