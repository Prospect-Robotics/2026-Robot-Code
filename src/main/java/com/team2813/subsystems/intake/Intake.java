package com.team2813.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged replayedInputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    Logger.processInputs("Intake", replayedInputs);
  }

  public void intake() {
    io.setExtensionSetpoint(IntakeConstants.getExtendOutSetpoint());
    io.setIntakeVoltage(IntakeConstants.getIntakeMotorVoltage());
  }

  public void outtake() {
    io.setExtensionSetpoint(IntakeConstants.getExtendInSetpoint());
    io.setIntakeVoltage(IntakeConstants.getOuttakeMotorVoltage());
  }

  public void stop() {
    io.setIntakeVoltage(Volts.of(0));
    io.stopExtender();
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
