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
    public Voltage rollerMotorVoltage = Volts.of(0);
    public AngularVelocity rollerMotorRPS = RotationsPerSecond.of(0);
    public Current rollerMotorCurrent = Amps.of(0);

    public Voltage rightFeederVoltage = Volts.of(0);
    public AngularVelocity rightFeederRPS = RotationsPerSecond.of(0);
    public Current rightFeederCurrent = Amps.of(0);

    public Voltage leftFeederVoltage = Volts.of(0);
    public AngularVelocity leftFeederRPS = RotationsPerSecond.of(0);
    public Current leftFeederCurrent = Amps.of(0);
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
   * @param rightFeederVoltage Voltage to apply to the right feeder motor.
   * @param leftFeederVoltage Voltage to apply to the left feeder motor.
   *     <p>Motor side (left/right) is based on when robot is viewed from behind).
   */
  default void setMotorVoltage(
      Voltage rollerVoltage, Voltage rightFeederVoltage, Voltage leftFeederVoltage) {}
}
