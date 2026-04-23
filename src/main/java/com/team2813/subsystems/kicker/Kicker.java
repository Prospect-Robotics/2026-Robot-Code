package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

/**
 * The kicker wheel that brings fuel that has gone through the indexer into the shooter. {@link
 * #shootCommand()} should be used when fuel needs to be brought into the shooter, while {@link
 * #outtakeCommand()} should be used when you want to resist the flow of fuel into the shooter.
 */
public class Kicker extends SubsystemBase implements AutoCloseable {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged replayedInputs;

  public Kicker(KickerIO io) {
    this.io = Objects.requireNonNull(io, "[Kicker] \"io\" cannot be null!");
    this.replayedInputs = new KickerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // could put somewhere else, but all the other code updates preferences once per cycle anyway
    io.updateState(replayedInputs);

    Logger.processInputs("Kicker", replayedInputs);
  }

  private void shoot() {
    io.setMotorVoltage(KickerConstants.getShootVoltage());
  }

  private void outtake() {
    io.setMotorVoltage(KickerConstants.getManualVoltage().times(-1));
  }
  private void intake() {
    io.setMotorVoltage(KickerConstants.getManualVoltage());
  }
  public void stop() {
    io.setMotorVoltage(Volts.of(0));
  }

  /**
   * Creates a command to shoot fuel. This should be used to bring fuel into the shooter when the
   * shooter is spooled up. This command will run indefinitely, and must be canceled or interrupted
   * to stop the kicker.
   *
   * @return The command to shoot fuel
   */
  public Command shootCommand() {
    return new StartEndCommand(this::shoot, this::stop, this);
  }

  /**
   * Creates a command to resist fuel going into the shooter. This command will run indefinitely,
   * and must be canceled or interrupted to stop the kicker.
   *
   * @return The command to resist fuel
   */
  public Command outtakeCommand() {
    return new StartEndCommand(this::outtake, this::stop, this);
  }
  public Command intakeCommand() {
    return new StartEndCommand(this::intake,this::stop,this);
  }
  /**
   * Creates a command to run the kicker with a custom voltage. This command will run indefinitely,
   * and must be canceled or interrupted to stop the kicker.
   *
   * @param voltageToRun The voltage to run the kicker at. Positive voltage runs the kicker in the
   *     direction of shooting.
   * @return A command to run the kicker at the desired voltage.
   */
  public Command customVoltageCommand(Voltage voltageToRun) {
    return new StartEndCommand(() -> io.setMotorVoltage(voltageToRun), this::stop, this);
  }

  @Override
  public void close() {
    io.close();
  }
}
