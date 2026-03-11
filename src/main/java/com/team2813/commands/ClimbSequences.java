package com.team2813.commands;

import com.team2813.subsystems.climb.Climb;
import com.team2813.subsystems.climb.ClimbConstants;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.Supplier;

public class ClimbSequences {

  private ClimbSequences() {}

  /**
   * This should be added as a default command to each Climb, through {@link
   * Climb#setDefaultCommand(Command)}
   *
   * @param joystickAxis A double supplier to the controller axis to manually control the climb.
   * @param climb Instance of climb to apply this command to
   * @return A command to be set as the default command of the given climb instance (preferably the
   *     outer).
   */
  public static Command getOuterClimbVoltageManualCommand(
      Supplier<Double> joystickAxis, Climb climb) {
    Supplier<Voltage> motorVoltSetpoint =
        () -> ClimbConstants.MANUAL_OUTER_CLIMB_VOLTAGE.times(joystickAxis.get());

    // I believe this needs to be a supplier, so we can update the voltage in realtime, rather than
    // it just being constant.
    return new RunCommand(() -> climb.setMotorVoltage(motorVoltSetpoint.get()), climb);
  }
}
