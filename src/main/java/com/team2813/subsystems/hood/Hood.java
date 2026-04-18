package com.team2813.subsystems.hood;

import static com.team2813.subsystems.hood.HoodConstants.HOOD_MOVEMENT_TIMEOUT;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private HoodIOInputsAutoLogged replayedInputs;

  // Note: This variable may be a little delayed to the actual position.
  private boolean hoodAtPosition = true;

  public Hood(HoodIO io) {
    this.io = io;
    this.replayedInputs = new HoodIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    Logger.processInputs("Hood", replayedInputs);
    io.updateState(replayedInputs);

    double hoodError = replayedInputs.motorAngle.minus(replayedInputs.motorSetpoint).abs(Rotations);

    hoodAtPosition = hoodError <= HoodConstants.ACCEPTABLE_MOTOR_ERROR.in(Rotations);

    Logger.recordOutput("Hood/currentHoodAngleDegrees", getCurrentHoodAngle().in(Degrees));
    Logger.recordOutput("Hood/atPosition", hoodAtPosition);
  }

  /**
   * @param angle Angle for the hood to move to.
   * @return A start end command that stops the motor on completion.
   */
  public Command goToAngleCommand(Angle angle) {
    //    atPosition = withinAcceptableErrorCalculation();
    return new StartEndCommand(() -> goToAngle(angle), this::stopMotor, this)
        .until(this::isHoodAtPosition)
        .withTimeout(HOOD_MOVEMENT_TIMEOUT);
  }

  /**
   * Wrapper for the {@link HoodIO#setSetpoint(Angle)} method, taking into account the.
   *
   * @param angle Angle of the <b>HOOD</b> to move to.
   */
  public void goToAngle(Angle angle) {
    hoodAtPosition = false;
    Angle currentSetpoint = angle;
    Logger.recordOutput("Hood/Setpoint", currentSetpoint);
    io.setSetpoint(angle.times(HoodConstants.HOOD_GEAR_RATIO));
  }

  public boolean isHoodAtPosition() {
    return hoodAtPosition;
  }

  /**
   * @return The current angle of the hood (likely in radians).
   */
  public Angle getCurrentHoodAngle() {
    return replayedInputs.motorAngle.div(HoodConstants.HOOD_GEAR_RATIO);
  }

  /** Stops the motor in its current position, causing it to brake and resist motion. */
  public void stopMotor() {
    io.stop();
  }
}
