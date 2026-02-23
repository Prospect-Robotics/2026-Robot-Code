package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    io.setShooterMotorVoltage(Volts.of(0));
  }

  public Command spoolShooterInstantIntakeCommand() {
    return new InstantCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterIntakeVoltage()), this);
  }

  public Command spoolShooterIntakewardCommand() {
    return new StartEndCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterIntakeVoltage()),
        this::stop,
        this);
  }

  public Command outakeCommand() {
    return new StartEndCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterOuttakeVoltage()), this::stop);
  }

  // Instructions taken from https://docs.advantagekit.org/data-flow/sysid-compatibility/ and
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
  public Command sysIDRoutine() {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIDTestState", state.toString())),
            new SysIdRoutine.Mechanism(io::setShooterMotorVoltage, null, this));
    // NOTE(spderman3333): I may need to use this::setShooterMotorVoltage rather than
    // io::setShooterMotorVoltage.

    return new SequentialCommandGroup(
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  // Used for auto calculated motor speed.
  public void setShooterMotorVoltage(Voltage voltage) {
    io.setShooterMotorVoltage(voltage);
  }
}
