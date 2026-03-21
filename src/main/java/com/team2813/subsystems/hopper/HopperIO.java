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
    public Voltage mainRollerMotorVoltage = Volts.of(0);
    public AngularVelocity mainRollerMotorRPS = RotationsPerSecond.of(0);
    public Current mainRollerMotorStatorCurrent = Amps.of(0);
    public Current mainRollerMotorSupplyCurrent = Amps.of(0);

    public Voltage followerRollerMotorVoltage = Volts.of(0);
    public AngularVelocity followerRollerMotorRPS = RotationsPerSecond.of(0);
    public Current followerRollerMotorStatorCurrent = Amps.of(0);
    public Current followerRollerMotorSupplyCurrent = Amps.of(0);

    // Feeder/Vector
    public Voltage feederVoltage = Volts.of(0);
    public AngularVelocity feederRPS = RotationsPerSecond.of(0);
    public Current feederStatorCurrent = Amps.of(0);
    public Current feederSupplyCurrent = Amps.of(0);
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
