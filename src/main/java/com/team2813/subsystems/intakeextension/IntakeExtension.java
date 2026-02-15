package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends SubsystemBase {
  private final IntakeExtensionIO io;
  private final IntakeExtensionIOInputsAutoLogged replayedInputs =
      new IntakeExtensionIOInputsAutoLogged();
  private boolean extenderAtPosition = true;

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
    extenderAtPosition =
        error <= 0.5; // Is the error between the setpoint greater than half a rotation.

    Logger.recordOutput("IntakeExtension/extenderAtPosition", extenderAtPosition);
    Logger.processInputs("IntakeExtension", replayedInputs);
  }

  public boolean isExtenderAtPosition() {
    return extenderAtPosition;
  }

  public void extend() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(IntakeExtensionConstants.getExtendOutSetpoint());
  }

  public void retract() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(IntakeExtensionConstants.getExtendInSetpoint());
  }

  public void setExtenderVoltage(Voltage extensionVoltage) {
    io.setExtenderVoltage(extensionVoltage);
  }

  public void stopExtender() {
    io.setExtenderVoltage(Volts.of(0));
  }

  public Command extendCommand() {
    return new InstantCommand(this::extend, this);
  }

  public Command retractCommand() {
    return new InstantCommand(this::retract, this);
  }

  public Command stopExtenderCommand() {
    return new InstantCommand(this::stopExtender, this);
  }
}
