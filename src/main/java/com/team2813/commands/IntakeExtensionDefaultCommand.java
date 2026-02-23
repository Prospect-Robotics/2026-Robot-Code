package com.team2813.commands;

import static edu.wpi.first.units.Units.Volts;

import com.team2813.subsystems.intakeextension.IntakeExtension;
import com.team2813.subsystems.intakeextension.IntakeExtensionConstants;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class IntakeExtensionDefaultCommand extends Command {
  private final IntakeExtension intakeExtension;
  private final DoubleSupplier movement;

  public IntakeExtensionDefaultCommand(IntakeExtension intakeExtension, DoubleSupplier movement) {
    this.intakeExtension = intakeExtension;
    this.movement = movement;
    addRequirements(intakeExtension);
  }

  @Override
  public void execute() {
    double val = movement.getAsDouble();
    if (Math.abs(val) > 0.1) {
      // Set the voltage, potentially disabling the PID controller
      intakeExtension.setExtenderVoltage(
          Volts.of(val * IntakeExtensionConstants.MANUAL_SPEED_FACTOR));
    } else if (!intakeExtension.isPidControlEnabled()) {
      // Manual control was initiated, but has stopped; stop the motor
      intakeExtension.stopMotor();
    }
  }
}
