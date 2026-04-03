package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase implements AutoCloseable {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged replayedInputs = new HoodIOInputsAutoLogged();
  private boolean atPosition = true;

  public Hood(HoodIO io) {
    this.io = Objects.requireNonNull(io, "io");

    // initialize preferences
    Preferences.initDouble(HoodConstants.HUB_ANGLE_PREFERENCE, HoodConstants.DEFAULT_HUB_ANGLE);
    Preferences.initDouble(
        HoodConstants.TRENCH_ANGLE_PREFERENCE, HoodConstants.DEFAULT_TRENCH_ANGLE);
    updatePreferences();
  }

  /**
   * Creates a command to bring the variable hood to the specified angle. This angle is the angle
   * that should be shot at. After this command finishes executing normally, the hood will be at the
   * requested angle, and stay there until another angle is requested, or {@link #neutralCommand()}
   * puts the hood into neutral mode.
   *
   * @param angle The angle to move the hood to
   * @return A command to bring the hood to the specified angle
   * @see #gotoAngleCommand(Supplier)
   */
  public Command gotoAngleCommand(Angle angle) {
    return new StartEndCommand(() -> gotoAngle(angle), () -> {}, this).until(this::atPosition);
  }

  /**
   * Creates a command to bring the variable hood to the angle returned by the given supplier when
   * it is scheduled. This angle is the angle that should be shot at. After this command finishes
   * executing normally, the hood will be at the requested angle, and stay there until another angle
   * is requested, or {@link #neutralCommand()} puts the hood into neutral mode.
   *
   * @param angleSupplier A supplier of the angle to move the hood to
   * @return A command to bring the hood to the specified angle
   * @see #gotoAngleCommand(Angle)
   */
  public Command gotoAngleCommand(Supplier<Angle> angleSupplier) {
    return new DeferredCommand(() -> gotoAngleCommand(angleSupplier.get()), Set.of(this));
  }

  /**
   * Creates a command to put the hood into neutral mode. In neutral mode, the hood will stop
   * attempting to stay at the last requested position, and let gravity move the hood down. This
   * state will end upon {@link #gotoAngleCommand(Angle)} or {@link #gotoAngleCommand(Supplier)}
   * gives the hood another angle to go to.
   *
   * @return A command that puts the hood into neutral mode
   */
  public Command neutralCommand() {
    return new InstantCommand(io::neutral, this);
  }

  private void gotoAngle(Angle angle) {
    io.setSetpoint(transformAngle(angle));
  }

  public boolean atPosition() {
    return atPosition;
  }

  /**
   * Transform an angle between the angle to shoot and the angle of the shooter. This operation is
   * symmetrical, so inputting an angle from either reference point will give the angle of the
   * other. The behavior of this function is undefined if the angle provided is not an angle that
   * can be reached physically.
   *
   * @param angle The angle in either reference point
   * @return The angle in the other reference point
   */
  private Angle transformAngle(Angle angle) {
    return Radians.of(Math.PI / 2).minus(HoodConstants.MINIMUM_SHOOTER_ANGLE).minus(angle);
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    double error = replayedInputs.motorAngle.minus(replayedInputs.motorSetpoint).abs(Radians);

    atPosition = error < Math.PI;
    Logger.recordOutput("Hood/atPosition", atPosition);
    Logger.recordOutput("Hood/shootAngle", transformAngle(replayedInputs.motorAngle));
    Logger.processInputs("Hood", replayedInputs);
  }

  private double currentHubAngle = HoodConstants.DEFAULT_HUB_ANGLE;
  private double currentTrenchAngle = HoodConstants.DEFAULT_TRENCH_ANGLE;
  private final Alert hubAngleAlert = new Alert(createAlertMessage("hubAngle"), AlertType.kInfo);
  private final Alert trenchAngleAlert =
      new Alert(createAlertMessage("trenchAngle"), AlertType.kInfo);

  /**
   * Get the angle required for hub shooting. This angle can directly be passed to {@link
   * #gotoAngleCommand(Angle)}.
   *
   * @return The angle for shooting at the hub
   */
  public Angle hubAngle() {
    return Degrees.of(currentHubAngle);
  }

  /**
   * Get the angle required for trench shooting. This angle can directly be passed to {@link
   * #gotoAngleCommand(Angle)}.
   *
   * @return The angle for shooting in the trench
   */
  public Angle trenchAngle() {
    return Degrees.of(currentTrenchAngle);
  }

  /**
   * Sets {@link #currentHubAngle} and {@link #currentTrenchAngle} to reflect the current preference
   * values. Additionally, puts up alerts if the preference value does not match the default value.
   */
  private void updatePreferences() {
    // Get new preference values, and set the alerts to pop up if they aren't the default value
    currentHubAngle = Preferences.getDouble(HoodConstants.HUB_ANGLE_PREFERENCE, currentHubAngle);
    hubAngleAlert.set(currentHubAngle != HoodConstants.DEFAULT_HUB_ANGLE);
    currentTrenchAngle =
        Preferences.getDouble(HoodConstants.TRENCH_ANGLE_PREFERENCE, currentTrenchAngle);
    trenchAngleAlert.set(currentTrenchAngle != HoodConstants.DEFAULT_TRENCH_ANGLE);
  }

  private static String createAlertMessage(String preference) {
    return String.format(
        "[HOOD] The %s was changed in Preferences! Once you are done tuning, please update the code!",
        preference);
  }

  @Override
  public void close() {
    io.close();
  }
}
