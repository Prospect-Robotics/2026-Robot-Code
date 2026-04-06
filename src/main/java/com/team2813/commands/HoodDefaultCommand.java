package com.team2813.commands;

import com.team2813.subsystems.hood.Hood;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** Command class to allow the operator to manually control the hood position using a joystick */
public class HoodDefaultCommand extends Command {
  Hood hoodInstance;
  Supplier<Double> doubleSupplier;

  // This will save on making a new Angle mesure everytime, as Angle is immutable by default.
  MutableMeasure<AngleUnit, Angle, MutAngle> currentHoodSetpoint;
  AngleUnit unitOfSetpoint;

  // How much to multiply the joystick double to be added to angle.
  private static double joystickMultiplier = 5;

  /**
   * @param hoodInstance A reference to the main instance of {@link Hood} in {@link
   *     com.team2813.RobotContainer}
   * @param doubleSupplier A method reference to a double supplier like a controller's joystick.
   */
  public HoodDefaultCommand(Hood hoodInstance, Supplier<Double> doubleSupplier) {
    this.hoodInstance = hoodInstance;
    this.doubleSupplier = doubleSupplier;
    // Line 26: It may be overkill, could just be Rotations.of(0)
    currentHoodSetpoint.mut_replace(hoodInstance.getCurrentHoodMotorAngle());
    unitOfSetpoint = hoodInstance.getCurrentHoodMotorAngle().unit();
  }

  @Override
  public void execute() {
    currentHoodSetpoint.mut_plus(joystickMultiplier * doubleSupplier.get(), unitOfSetpoint);
    hoodInstance.goToAngleCommand(currentHoodSetpoint.copy());
  }
}
