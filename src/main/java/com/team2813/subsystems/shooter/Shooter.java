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
    io.setMotorVoltages(Volts.of(0), Volts.of(0));
  }

  // Waits before starting the kicker to allow the shooter flywheel to get up to speed.
  //  public Command intakeCommand() {
  //    return new SequentialCommandGroup(
  //            new ParallelCommandGroup(
  //                new InstantCommand(
  //                    () ->
  // io.setShooterMotorVoltage(ShooterConstants.getShooterIntakeVoltage())),
  //                new InstantCommand(
  //                    () ->
  // io.setKickerMotorVoltage(ShooterConstants.getKickerOuttakeVoltage()))),
  //            new WaitCommand(Seconds.of(2)),
  //            new StartEndCommand(
  //                () -> io.setKickerMotorVoltage(ShooterConstants.getKickerIntakeVoltage()),
  //                this::stop,
  //                this))
  //        .finallyDo(this::stop);
  //    /*
  //    Note: I still use StartEndCommand because it only calls the first Runnable once,
  //     rather than repeatedly like RunCommand.
  //     Thus I believe it will save on hardware calls, but it could just be over engineering.
  //     */
  //  }\

  public Command spoolShooterInstantIntakeCommand() {
    return new InstantCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterIntakeVoltage()));
  }

  public Command spoolShooterIntakewardCommand() {
    return new StartEndCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterIntakeVoltage()),
        this::stop,
        this);
  }

  public Command runKickerIntakewardCommand() {
    return new StartEndCommand(
        () -> io.setKickerMotorVoltage(ShooterConstants.getKickerIntakeVoltage()),
        this::stop,
        this);
  }

  public Command outakeCommand() {
    return new StartEndCommand(
        () ->
            io.setMotorVoltages(
                ShooterConstants.getShooterOuttakeVoltage(),
                ShooterConstants.getKickerOuttakeVoltage()),
        this::stop);
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

  public void setKickerMotorVoltage(Voltage kickerMotorVoltage) {
    io.setKickerMotorVoltage(kickerMotorVoltage);
  }

  // Used for auto calculated motor speed.
  public void setShooterMotorVoltage(Voltage voltage) {
    io.setShooterMotorVoltage(voltage);
  }
}
