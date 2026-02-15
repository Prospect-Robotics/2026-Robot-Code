package com.team2813.commands;

import static edu.wpi.first.units.Units.Volts;

import com.team2813.subsystems.intake.Intake;
import com.team2813.subsystems.intake.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class IntakeExtensionDefaultCommand extends Command {
  private final Intake intake;
  private final DoubleSupplier movement;

  public IntakeExtensionDefaultCommand(Intake intake, DoubleSupplier movement) {
    this.intake = intake;
    this.movement = movement;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    double val = movement.getAsDouble();
    // manual control is being used
    intake.setExtenderVoltage(Volts.of(val * IntakeConstants.MANUAL_SPEED_FACTOR));
  }
}
