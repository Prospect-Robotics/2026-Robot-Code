package com.team2813.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Interface to allow the subsystem to control the motors of the subsystem ( */
public interface HopperIO {

  @AutoLog
  class HopperIOInputs {
    // Roller/Magazine
    public Voltage feederMotorVoltage = Volts.of(0);
    public AngularVelocity feederMotorRPS = RotationsPerSecond.of(0);
    public Current feederMotorStatorCurrent = Amps.of(0);
    public Current feederMotorSupplyCurrent = Amps.of(0);
  }

  /**
   * Updates Advantage kit autologged input data, as well as any other necessary states (like in
   * sim)
   *
   * @param inputs The "struct" (data class) to handle hardware inputs.
   */
  default void updateState(HopperIOInputs inputs) {}

  /**
   * @param rollerVoltage Voltage to apply to the roller motor.
   * @param feederVoltage Voltage to apply to the feeder motor.
   */
  default void setMotorVoltage(Voltage rollerVoltage, Voltage feederVoltage) {}
}
