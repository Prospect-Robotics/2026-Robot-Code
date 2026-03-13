package com.team2813.commands;

import com.team2813.subsystems.climb.AllClimbs;
import com.team2813.subsystems.climb.Climb;
import com.team2813.subsystems.climb.ClimbConstants;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

// TODO: Lets not make this a static class.
/** A collections of commands to run on the climb. */
public class ClimbSequences {

  private ClimbSequences() {}

  /**
   * This should be added as a default command to each Climb, through {@link
   * Climb#setDefaultCommand(Command)}
   *
   * @param joystickAxis A double supplier to the controller axis to manually control the climb.
   * @param innerClimbInstance Instance of climb to apply this command to (preferably inner climb).
   * @return A command to be set as the default command of the given climb instance (preferably the
   *     outer).
   */
  public static Command innerClimbManualCommand(
      Supplier<Double> joystickAxis, Climb innerClimbInstance) {
    Supplier<Voltage> motorVoltSetpoint =
        () -> ClimbConstants.MANUAL_INNER_CLIMB_VOLTAGE.times(joystickAxis.get());

    // I believe this needs to be a supplier, so we can update the voltage in realtime, rather than
    // it just being constant.
    return new RunCommand(
        () -> innerClimbInstance.setMotorVoltage(motorVoltSetpoint.get()), innerClimbInstance);
  }

  /**
   * This should be bound to a {@link
   * edu.wpi.first.wpilibj2.command.button.Trigger#whileTrue(Command)}
   *
   * @param innerClimbInstance The instance of inner climb to apply this command to.
   * @return A {@link StartEndCommand} to set the motor voltage to go upward with 3 volts.
   */
  public static Command innerClimbManualUpCommand(Climb innerClimbInstance) {
    return new StartEndCommand(
        () -> innerClimbInstance.setMotorVoltage(ClimbConstants.MANUAL_INNER_CLIMB_VOLTAGE),
        innerClimbInstance::stopClimb,
        innerClimbInstance);
  }

  /**
   * This should be bound to a {@link
   * edu.wpi.first.wpilibj2.command.button.Trigger#whileTrue(Command)}.
   *
   * @param innerClimbInstance The instance of inner climb to apply this command to.
   * @return A {@link StartEndCommand} to set the motor voltage to go downward with 3 volts.
   */
  public static Command innerClimbManualDownCommand(Climb innerClimbInstance) {
    return new StartEndCommand(
        () ->
            innerClimbInstance.setMotorVoltage(
                ClimbConstants.MANUAL_INNER_CLIMB_VOLTAGE.unaryMinus()),
        innerClimbInstance::stopClimb,
        innerClimbInstance);
  }

  /**
   * Moves the inner climb to its post auto position.
   *
   * <p>The climbInterruption is used for when the climb reaches its setpoint, or if the operator
   * interrupts via manual movement.
   *
   * @param innerClimbInstance The inner climb instance to apply the command to.
   * @param climbInterruption A boolean supplier to allow for cancellation of this command.
   * @return
   */
  public static Command postAutoClimb(Climb innerClimbInstance, BooleanSupplier climbInterruption) {
    return new StartEndCommand(
            () -> innerClimbInstance.setClimbPosition(AllClimbs.InnerClimbHeight.POSTAUTO),
            innerClimbInstance::stopClimb,
            innerClimbInstance)
        .until(climbInterruption);
  }
}
