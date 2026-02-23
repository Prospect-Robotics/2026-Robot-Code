package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase implements AutoCloseable {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged replayedInputs;
  private double shootVoltage = KickerConstants.SHOOT_VOLTAGE;
  private double resistFuelVoltage = KickerConstants.RESIST_FUEL_VOLTAGE;
  // note: could put alerts in a kicker-specific location, but it will be easier for seeing alerts
  // to put them all in the same place
  // also, the default behavior is to not show the alert
  private final Alert shootVoltageWarning =
      new Alert(createAlertMessage("shoot voltage"), Alert.AlertType.kInfo);
  private final Alert resistFuelVoltageWarning =
      new Alert(createAlertMessage("resist fuel voltage"), Alert.AlertType.kInfo);

  private static String createAlertMessage(String preference) {
    return String.format(
        "[KICKER] The %s was changed in preferences! Once you are done tuning, please update the code!",
        preference);
  }

  public Kicker(KickerIO io) {
    this.io = Objects.requireNonNull(io, "io");
    this.replayedInputs = new KickerIOInputsAutoLogged();

    Preferences.initDouble(KickerConstants.SHOOT_PREFERENCE_NT, shootVoltage);
    Preferences.initDouble(KickerConstants.RESIST_FUEL_PREFERENCE_NT, resistFuelVoltage);
    shootVoltageWarning.set(false);
    resistFuelVoltageWarning.set(false);
  }

  @Override
  public void periodic() {
    // could put somewhere else, but all the other code updates preferences once per cycle anyway
    updatePreferences();
    io.updateState(replayedInputs);

    Logger.processInputs("Kicker", replayedInputs);
  }

  private void shoot() {
    io.setMotorVoltage(Volts.of(shootVoltage));
  }

  private void resistFuel() {
    io.setMotorVoltage(Volts.of(resistFuelVoltage));
  }

  private void stop() {
    io.setMotorVoltage(Volts.of(0));
  }

  /**
   * Creates a command to shoot fuel. This command will run indefinitely, and must be canceled or
   * interrupted to stop the kicker.
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
  public Command resistFuelCommand() {
    return new StartEndCommand(this::resistFuel, this::stop, this);
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

  /**
   * Refresh all values from preferences. This will also put alerts onto NetworkTables if the value
   * from preferences does not match the value in code to encourage keeping the code up-to-date.
   */
  // note: if we update preferences somewhere else, we may need to change the visibility of this.
  private void updatePreferences() {
    shootVoltage = Preferences.getDouble(KickerConstants.SHOOT_PREFERENCE_NT, shootVoltage);
    shootVoltageWarning.set(shootVoltage != KickerConstants.SHOOT_VOLTAGE);
    resistFuelVoltage =
        Preferences.getDouble(KickerConstants.RESIST_FUEL_PREFERENCE_NT, resistFuelVoltage);
    resistFuelVoltageWarning.set(resistFuelVoltage != KickerConstants.RESIST_FUEL_VOLTAGE);
  }

  @Override
  public void close() {
    io.close();
  }
}
