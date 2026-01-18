package com.team2813.subsystems.shooter;

import com.team2813.subsystems.hopper.HopperIO;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {}

  /**
   * Updates Advantage kit autologged input data, as well as any other necessary states (like in
   * sim)
   *
   * @param inputs The "struct" (data class) to handle hardware inputs.
   */
  default void updateState(HopperIO.HopperIOInputs inputs) {}

  /**
   * @param voltage Voltage to apply to the motor.
   */
  default void setMotorVoltage(Voltage voltage) {}
}
