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
    public Voltage mainFeederMotorVoltage = Volts.of(0);
    public AngularVelocity mainFeederMotorRPS = RotationsPerSecond.of(0);
    public Current mainFeederMotorStatorCurrent = Amps.of(0);
    public Current mainFeederMotorSupplyCurrent = Amps.of(0);

    public Voltage followerFeederMotorVoltage = Volts.of(0);
    public AngularVelocity followerFeederMotorRPS = RotationsPerSecond.of(0);
    public Current followerFeederMotorStatorCurrent = Amps.of(0);
    public Current followerFeederMotorSupplyCurrent = Amps.of(0);

    // Feeder/Vector
    public Voltage indexerMotorVoltage = Volts.of(0);
    public AngularVelocity indexerMotorRPS = RotationsPerSecond.of(0);
    public Current indexerMotorStatorCurrent = Amps.of(0);
    public Current indexerMotorSupplyCurrent = Amps.of(0);
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
