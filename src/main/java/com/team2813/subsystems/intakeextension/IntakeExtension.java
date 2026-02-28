package com.team2813.subsystems.intakeextension;

import static com.team2813.subsystems.intakeextension.IntakeExtensionConstants.toIntakeExtensionPosition;
import static edu.wpi.first.units.Units.*;

import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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

  public IntakeExtension(IntakeExtensionIO io) {
    this.io = io;
  }

  /**
   * Allows manual control of this intake extension using a controller.
   *
   * @param controller supplier that supplies the value of the controller.
   */
  public void setManualOverrideController(DoubleSupplier controller) {
    Supplier<Voltage> voltageSupplier =
        () -> {
          double val = controller.getAsDouble();
          return Volts.of(val * IntakeExtensionConstants.MANUAL_SPEED_FACTOR);
        };

    Command setExtenderVoltageCommand =
        Commands.run(() -> io.setExtenderVoltage(voltageSupplier.get()), this);
    setDefaultCommand(setExtenderVoltageCommand);
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

  public Command extendCommand() {
    return Commands.startEnd(this::extend, this::stopMotor, this);
  }

  public Command retractCommand() {
    return Commands.startEnd(this::retract, this::stopMotor, this);
  }

  /**
   * Creates a command that moves the intake about halfway, used for Wall-E mode, as we retract to
   * this position (rather than fully retracting).
   */
  public Command halfRetractCommand() {
    return Commands.startEnd(this::halfRetract, this::stopMotor, this);
  }

  void extend() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT));
  }

  void retract() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.IN));
  }

  /**
   * Moves the intake about halfway, used for Wall-E mode, as we retract to this position (rather
   * than fully retracting).
   */
  private void halfRetract() {
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
                    halfRetractCommand().until(this::isExtenderAtPosition), extendCommand())
                .until(this::isExtenderAtPosition))
        .finallyDo(this::stopMotor);
  }

  public void stopMotor() {
    io.setExtenderVoltage(Volts.of(0));
  }

  public Angle getSetpoint() {
    return replayedInputs.extenderMotorSetpoint;
  }
}
