package com.team2813.subsystems.intakeextension;

import static com.team2813.subsystems.intakeextension.IntakeExtensionConstants.toIntakeExtensionPosition;
import static edu.wpi.first.units.Units.*;

import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.units.Units;
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
public class IntakeExtension extends SubsystemBase implements AutoCloseable {
  static final double ACCEPTABLE_ERROR_IN_ROTATIONS = 0.4;
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
            .abs(Units.Rotation);
    extenderAtPosition = error <= ACCEPTABLE_ERROR_IN_ROTATIONS;

    Logger.recordOutput("IntakeExtension/extenderAtPosition", extenderAtPosition);
    Logger.recordOutput(
        "IntakeExtension/PositionInRotations", replayedInputs.extenderMotorPosition.in(Rotations));
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
    setSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT);
  }

  void retract() {
    setSetpoint(IntakeExtensionConstants.ExtenderPositions.IN);
  }

  /**
   * Moves the intake about halfway, used for Wall-E mode, as we retract to this position (rather
   * than fully retracting).
   */
  private void halfRetract() {
    setSetpoint(IntakeExtensionConstants.ExtenderPositions.MIDDLE);
  }

  private void setSetpoint(IntakeExtensionConstants.ExtenderPositions position) {
    extenderAtPosition = false;
    Angle setpoint = position.getAngle();
    Logger.recordOutput("IntakeExtension/Setpoint", setpoint);
    io.setExtensionSetpoint(setpoint);
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
                halfRetractCommand().until(this::isExtenderAtPosition),
                new WaitCommand(0.5),
                extendCommand().until(this::isExtenderAtPosition),
                new WaitCommand(0.5)))
        .finallyDo(this::stopMotor);
  }

  public void stopMotor() {
    io.setExtenderVoltage(Volts.of(0));
  }

  public Angle getSetpoint() {
    return replayedInputs.extenderMotorSetpoint;
  }

  public Angle getPosition() {
    return replayedInputs.extenderMotorPosition;
  }

  @Override
  public void close() throws Exception {
    io.close();
  }
}
