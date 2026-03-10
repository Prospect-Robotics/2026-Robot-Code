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
    public double innerCarriagePositionInches = 0.0;
    // public double motorSetpointRotations = 0.0;
    public double innerMotorRotations = 0.0;
    public AngularVelocity innerMotorVelocityRotsPerSecond = RotationsPerSecond.of(0.0);
    public double innerMotorCurrent = 0.0;
    public double innerMotorVoltage = 0.0;

    public double outerCarriagePositionInches = 0.0;
    // public double motorSetpointRotations = 0.0;
    public double outerMotorRotations = 0.0;
    public AngularVelocity outerMotorVelocityRotsPerSecond = RotationsPerSecond.of(0.0);
    public double outerMotorCurrent = 0.0;
    public double outerMotorVoltage = 0.0;
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
  void setInnerMotorSetpoint(Angle setpoint) {}

  void setOuterMotorSetpoint(Angle setpoint) {}

  /** Runs a brake request on the inner climb motor, stopping it and preventing movement. */
  void stopInnerMotor() {}

  /** Runs a brake request on the outer climb motor, stopping it and preventing movement. */
  void stopOuterMotor() {}

  /**
   * @return The angle of the motor.
   */
  Angle getInnerMotorPosition() {
    return Rotations.of(0);
  }

  Angle getOuterMotorPosition() {
    return Rotations.of(0);
  }

  /**
   * Needs to be calculated based off of the motor rotational position, gearing, and spool radius,
   *
   * @return The position of the climbs carriage.
   */
  Distance getInnerCarriagePosition() {
    return Meters.of(0);
  }

  Distance getOuterCarriagePosition() {
    return Meters.of(0);
  }

  void setInnerMotorVoltage(Voltage motorVoltage) {}

  void setOuterMotorVoltage(Voltage MotorVoltage) {}
}
