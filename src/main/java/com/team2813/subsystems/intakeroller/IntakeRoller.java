package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

/** Code that controls the rollers of the intake. */
public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged replayedInputs;

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
    replayedInputs = new IntakeRollerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    Logger.processInputs("IntakeRoller", replayedInputs);
  }

  // TODO: Private all of these methods, except, potentially stop? Will need to decide on how A-stopping will work.
  public void intake() {
    io.setIntakeMotorVoltage(IntakeRollerConstants.getIntakeVoltage());
  }

  public void outtake() {
    io.setIntakeMotorVoltage(IntakeRollerConstants.getOuttakeVoltage());
  }

  public void stop() {
    io.setIntakeMotorVoltage(Volts.of(0));
  }

  /**
   * This command should be used with Trigger.whileTrue();
   *
   * @return StartEndCommand instance which stops the motor on the end of the command.
   */
  public Command intakeCommand() {
    return new StartEndCommand(this::intake, this::stop, this);
  }

  public Command outtakeCommand() {
    return new StartEndCommand(this::outtake, this::stop, this);
  }

  // Used for autopaths.
  public Command stopCommand() {
    return new InstantCommand(this::stop, this);
  }
}
