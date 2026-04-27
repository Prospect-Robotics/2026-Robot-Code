package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
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

  public Command spoolShooterTrenchSpeedCommand() {
    return spoolCustomVelocityCommand(ShooterConstants.getShooterTrenchShootVelocity());
  }

  public Command spoolShooterHubSpeedCommand() {
    return spoolCustomVelocityCommand(ShooterConstants.getShooterHubShootVelocity());
  }

  public Command spoolShooterHerdSpeedCommand() {
    return spoolCustomVelocityCommand(ShooterConstants.getShooterHerdShootVelocity());
  }

  public Command spoolShooterTowerSpeedCommand() {
    return spoolCustomVelocityCommand(ShooterConstants.getShooterTowerShootVelocity());
  }

  public Command outakeCommand() {
    return new StartEndCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterOuttakeVoltage()), this::stop);
  }

  public Command spoolCustomVelocityCommand(AngularVelocity velocity) {
    return new StartEndCommand(() -> io.setShooterMotorVelocity(velocity), this::stop, this);
  }

  public Command idleCommand() {
    return new StartEndCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getIdleVoltage()), this::stop, this);
  }

  // Instructions taken from https://docs.advantagekit.org/data-flow/sysid-compatibility/ and
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
  public Command sysIDRoutine() {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(0.1),
                Volts.of(1),
                null,
                (state) -> Logger.recordOutput("SysIDTestState", state.toString())),
            new SysIdRoutine.Mechanism(io::setShooterMotorVoltage, null, this));
    // NOTE(spderman3333): I may need to use this::setShooterMotorVoltage rather than
    // io::setShooterMotorVoltage.

    return new SequentialCommandGroup(
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        new WaitCommand(5),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        new WaitCommand(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
        new WaitCommand(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  // Used for auto calculated motor speed.
  public void setShooterMotorVelocity(AngularVelocity velocity) {
    io.setShooterMotorVelocity(velocity);
  }

  /**
   * Used for automatically running kicker and hopper motors once the shooter is spooled to speed.
   *
   * @return <code>true</code> if the motor is within {@link
   *     ShooterConstants#SHOOTER_SPOOL_SPEED_TOLERANCE} of the current motor setpoint
   */
  public boolean isMotorVelocityWithinTolerance() {
    return RotationsPerSecond.of(replayedInputs.upperRightShooterMotorRotPerSec)
        .isNear(
            RotationsPerSecond.of(replayedInputs.upperRightShooterSetpointRotsPerSec),
            ShooterConstants.SHOOTER_SPOOL_SPEED_TOLERANCE);
  }
}
