package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public void intake() {
    io.setMotorVoltage(
        ShooterConstants.getShooterIntakeVoltage(), ShooterConstants.getKickerIntakeVoltage());
  }

  public void outtake() {
    io.setMotorVoltage(
        ShooterConstants.getShooterOuttakeVoltage(), ShooterConstants.getKickerOuttakeVoltage());
  }

  public void stop() {
    io.setMotorVoltage(Volts.of(0), Volts.of(0));
  }

  public Command intakeCommand() {
    return new InstantCommand(this::intake, this);
  }

  public Command outakeCommand() {
    return new InstantCommand(this::outtake, this);
  }

  public Command stopCommand() {
    return new InstantCommand(this::stop, this);
  }

  // Instructions taken from https://docs.advantagekit.org/data-flow/sysid-compatibility/ and
  public Command sysIDRoutine() {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIDTestState", state.toString())),
            new SysIdRoutine.Mechanism(io::setShooterMotorVoltage, null, this));
    // Note: I may need to use this::setShooterMotorVoltage rather than io::setShooterMotorVoltage.

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
