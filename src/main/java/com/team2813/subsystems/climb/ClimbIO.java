package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  class ClimbIOInputs {
    public double leftCarriagePositionInches = 0.0;
    // public double motorSetpointRotations = 0.0;
    public double leftMotorRotations = 0.0;
    public double leftMotorVelocityRotsPerSecond = 0.0;
    public double leftMotorCurrent = 0.0;
    public double leftMotorVoltage = 0.0;

    public double rightCarriagePositionInches = 0.0;
    // public double motorSetpointRotations = 0.0;
    public double rightMotorRotations = 0.0;
    public double rightMotorVelocityRotsPerSecond = 0.0;
    public double rightMotorCurrent = 0.0;
    public double rightMotorVoltage = 0.0;
  }

  /** Sets the TalonFX motor configuraiton in the IO implementation. */
  default void setMotor(TalonFX motor) {}

  /**
   * Updates Advantage kit autologged input data, as well as any other necessary states (like in
   * sim)
   *
   * @param inputs The "struct" (data class) to handle hardware inputs.
   */
  default void updateState(ClimbIOInputs inputs) {}

  /**
   * Uses positional control for the motor, using its internal PID values.
   *
   * @param setpoint Position for the motor to go to.
   */
  default void setMotorSetpoint(Angle setpoint) {}

  /**
   * @param voltage Voltage to apply to the motor.
   */
  default void setMotorVoltage(Voltage voltage) {}

  /**
   * @return The angle of the motor.
   */
  default Angle getMotorPosition() {
    return Rotations.of(0);
  }

  /**
   * Needs to be calculated based off of the motor rotational position, gearing, and spool radius,
   *
   * @return The position of the elevator's carriage.
   */
  default Distance getCarriagePosition() {
    return Meters.of(0);
  }
}
