package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Objects;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Code for the moving hood to angle the shot from the shooter. */
public class Hood extends SubsystemBase implements AutoCloseable {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged replayedInputs = new HoodIOInputsAutoLogged();
  private boolean isAtPosition = true;

  public Hood(HoodIO io) {
    this.io = Objects.requireNonNull(io, "io");

    // initialize preferences
  }

  /**
   * Creates a command to bring the variable hood to the specified angle. This angle is the angle
   * that should be shot at. After this command finishes executing normally, the hood will be at the
   * requested angle, and stay there until another angle is requested, or {@link #neutralCommand()}
   * puts the hood into neutral mode.
   *
   * @param angle The angle to move the hood to
   * @return A command to bring the hood to the specified angle
   * @see #goToAngleCommand(Supplier)
   */
  public Command goToAngleCommand(Angle angle) {
    return new StartEndCommand(() -> goToAngle(angle), () -> {}, this);
  }

  /**
   * Creates a command to bring the variable hood to the angle returned by the given supplier when
   * it is scheduled. This angle is the angle that should be shot at. After this command finishes
   * executing normally, the hood will be at the requested angle, and stay there until another angle
   * is requested, or {@link #neutralCommand()} puts the hood into neutral mode.
   *
   * @param angleSupplier A supplier of the angle to move the hood to
   * @return A command to bring the hood to the specified angle
   * @see #goToAngleCommand(Angle)
   */
  public Command goToAngleCommand(Supplier<Angle> angleSupplier) {
    return goToAngleCommand(angleSupplier.get());
  }

  /**
   * Creates a command to put the hood into neutral mode. In neutral mode, the hood will stop
   * attempting to stay at the last requested position, and let gravity move the hood down. This
   * state will end upon {@link #goToAngleCommand(Angle)} or {@link #goToAngleCommand(Supplier)}
   * gives the hood another angle to go to.
   *
   * @return A command that puts the hood into neutral mode
   */
  public Command neutralCommand() {
    return new InstantCommand(io::stop, this);
  }

  public void goToAngle(Angle angle) {
    io.setSetpoint(angle.times(HoodConstants.HOOD_GEAR_RATIO));
  }

  public boolean atPosition() {
    return isAtPosition;
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    double error = replayedInputs.motorAngle.minus(replayedInputs.motorSetpoint).abs(Radians);

    isAtPosition = error < Math.PI / 16;
    Logger.recordOutput("Hood/AtPostion", isAtPosition);
    Logger.recordOutput("Hood/PositionSetpointError", error);
    Logger.recordOutput("Hood/HoodAngle", replayedInputs.motorAngle);
    Logger.processInputs("Hood", replayedInputs);
  }

  /**
   * Get the angle required for hub shooting. This angle can directly be passed to {@link
   * #goToAngleCommand(Angle)}.
   *
   * @return The angle for shooting at the hub
   */
  public Angle hubAngle() {
    return Degrees.of(17);
  }

  /**
   * Get the angle required for trench shooting. This angle can directly be passed to {@link
   * #goToAngleCommand(Angle)}.
   *
   * @return The angle for shooting in the trench
   */
  public Angle trenchAngle() {
    return Degrees.of(40);
  }

  /**
   * @return The current motor position.
   */
  public Angle getCurrentHoodMotorAngle() {
    return replayedInputs.motorAngle;
  }

  @Override
  public void close() {
    io.close();
  }
}
