package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged replayedInputs = new HoodIOInputsAutoLogged();

  // Note: This variable may be a little delayed to the actual position.
  private boolean hoodAtPosition = true;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Hood", replayedInputs);
    io.updateState(replayedInputs);

    double hoodMotorAngleAbsError = replayedInputs.motorAngle.minus(replayedInputs.motorSetpoint).abs(Rotations);

    hoodAtPosition = hoodMotorAngleAbsError <= HoodConstants.ACCEPTABLE_MOTOR_ERROR.in(Rotations);

    Logger.recordOutput("Hood/currentHoodAngleDegrees", getCurrentHoodAngle().in(Degrees));
    Logger.recordOutput("Hood/atPosition", hoodAtPosition);
  }

  /**
   * @param angle Hood related angle for the hood to move to.
   * @return A start end command that stops the motor on completion.
   */
  public Command goToAngleCommand(Angle angle) {
    //    atPosition = withinAcceptableErrorCalculation();
    return new StartEndCommand(() -> goToAngle(angle), this::stopMotor, this)
        .until(this::isHoodAtPosition)
        .withTimeout(HoodConstants.HOOD_MOVEMENT_TIMEOUT);
  }

  /**
   * Wrapper for the {@link HoodIO#setSetpoint(Angle)} method, taking into account the.
   *
   * @param angle The angle in relation to the <b>HOOD</b> to for the hood move to.
   */
  public void goToAngle(Angle angle) {
    hoodAtPosition = false;
    Angle currentSetpoint = angle;
    Logger.recordOutput("Hood/Setpoint", currentSetpoint);
    io.setSetpoint(angle.times(HoodConstants.HOOD_GEAR_RATIO));
  }

  public Command sysIDRoutine() {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(0.1),
                Volts.of(1),
                null,
                (state) -> Logger.recordOutput("SysIDTestState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltage, null, this));
    // NOTE(spderman3333): I may need to use this::setShooterMotorVoltage rather than

    return new SequentialCommandGroup(
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        new WaitCommand(5),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        new WaitCommand(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
        new WaitCommand(5),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
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
