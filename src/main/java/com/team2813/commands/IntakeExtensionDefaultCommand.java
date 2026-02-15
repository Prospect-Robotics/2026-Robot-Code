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
    // manual control is being used
    intakeExtension.setExtenderVoltage(Volts.of(val * IntakeExtensionConstants.MANUAL_SPEED_FACTOR));
  }
}
