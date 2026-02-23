package com.team2813.subsystems.intakeextension;

import static com.team2813.subsystems.intakeextension.IntakeExtensionConstants.toIntakeExtensionPosition;
import static edu.wpi.first.units.Units.*;

import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

/**
 * Code that controls the extension of the intake and the front plate of the hopper via a rack and
 * pinion gear.
 */
public class IntakeExtension extends SubsystemBase {
  private final IntakeExtensionIO io;
  private final IntakeExtensionIOInputsAutoLogged replayedInputs =
      new IntakeExtensionIOInputsAutoLogged();
  private boolean extenderAtPosition = true;
  private boolean pidControlEnabled = false;

  public IntakeExtension(IntakeExtensionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    double error =
        replayedInputs
            .extenderMotorPosition
            .minus(replayedInputs.extenderMotorSetpoint)
            .abs(Rotation);

    // Is the error between the setpoint greater than half a rotation.
    extenderAtPosition = error <= 0.4;

    Logger.recordOutput("IntakeExtension/extenderAtPosition", extenderAtPosition);
    Logger.processInputs("IntakeExtension", replayedInputs);
  }

  @Override
  public void simulationPeriodic() {
    SimulationVisualizer.getInstance()
        .updateIntakeExtensionPosition(
            toIntakeExtensionPosition(replayedInputs.extenderMotorPosition));
  }

  public boolean isExtenderAtPosition() {
    return extenderAtPosition;
  }

  public void extend() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT));
    pidControlEnabled = true;
  }

  public void retract() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.IN));
    pidControlEnabled = true;
  }

  /**
   * Moves the intake about halfway, used for Wall-E mode, as we retract to this position (rather
   * than fully retracting).
   */
  public void halfRetract() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(
        IntakeExtensionConstants.toMotorSetpoint(
            IntakeExtensionConstants.ExtenderPositions.MIDDLE));
  }

  /**
   * Makes the intake extension repeatedly extend and retract in order to push balls toward the
   * shooter.
   *
   * @return A {@link RepeatCommand} that does the above.
   */
  public Command wallEMode() {
    return new RepeatCommand(
            new SequentialCommandGroup(
                    new StartEndCommand(this::halfRetract, this::stopMotor, this)
                        .until(this::isExtenderAtPosition),
                    new StartEndCommand(this::extend, this::stopMotor, this))
                .until(this::isExtenderAtPosition))
        .finallyDo(this::stopMotor);
  }

  public void setExtenderVoltage(Voltage extensionVoltage) {
    io.setExtenderVoltage(extensionVoltage);
    pidControlEnabled = false;
  }

  public void stopMotor() {
    io.setExtenderVoltage(Volts.of(0));
    pidControlEnabled = false;
  }

  public Angle getSetpoint() {
    return replayedInputs.extenderMotorSetpoint;
  }

  public boolean isPidControlEnabled() {
    return pidControlEnabled;
  }
}
