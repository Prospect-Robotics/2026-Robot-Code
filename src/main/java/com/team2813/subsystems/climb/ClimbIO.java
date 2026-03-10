package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public abstract class ClimbIO {

  AllClimbConstants climbConstants;

  @AutoLog
  class ClimbIOInputs {
    public double carriagePositionInches = 0.0;
    // public double motorSetpointRotations = 0.0;
    public double motorRotations = 0.0;
    public AngularVelocity motorVelocityRotsPerSecond = RotationsPerSecond.of(0.0);
    public double motorCurrent = 0.0;
    public double motorVoltage = 0.0;
  }

  public ClimbIO(AllClimbConstants climbConstants) {
    this.climbConstants = climbConstants;
  }

  /**
   * Updates Advantage kit autologged input data, as well as any other necessary states (like in
   * sim)
   *
   * @param inputs The "struct" (data class) to handle hardware inputs.
   */
  void updateState(ClimbIOInputs inputs) {}

  /**
   * Uses positional control for the motor, using its internal PID values.
   *
   * @param setpoint Position for the motor to go to.
   */
  void setMotorSetpoint(Angle setpoint) {}

  /** Runs a brake request on the climb motor, stopping it and preventing movement. */
  void stopMotor() {}

  /**
   * @return The angle of the motor.
   */
  Angle getMotorPosition() {
    return Rotations.of(0);
  }

  /**
   * Needs to be calculated based off of the motor rotational position, gearing, and spool radius,
   *
   * @return The position of the climbs carriage.
   */
  Distance getCarriagePosition() {
    return Meters.of(0);
  }

  void setMotorVoltage(Voltage MotorVoltage) {}
}
