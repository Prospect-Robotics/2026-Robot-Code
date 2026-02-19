package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim implements ClimbIO {

  // Physics sim for the elevator.
  private final ElevatorSim innerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.LEFTMOTOR_TO_CLIMB_GEARING,
          ClimbConstants.INNERCLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          ClimbConstants.INNERCLIMB_SPOOL_RADIUS.in(Meter),
          ClimbConstants.INNERCLIMB_MIN_HEIGHT.in(Meter),
          ClimbConstants.INNERCLIMB_MAX_HEIGHT.in(Meter),
          true,
          ClimbConstants.INNERCLIMB_MIN_HEIGHT.in(Meter));

  private final ElevatorSim outerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.RIGHTMOTOR_TO_CLIMB_GEARING,
          ClimbConstants.OUTERCLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          ClimbConstants.OUTERCLIMB_SPOOL_RADIUS.in(Meter),
          ClimbConstants.OUTERCLIMB_MIN_HEIGHT.in(Meter),
          ClimbConstants.OUTERCLIMB_MAX_HEIGHT.in(Meter),
          true,
          ClimbConstants.OUTERCLIMB_MIN_HEIGHT.in(Meter));

  private TalonFX motor;
  private TalonFXSimState motorSim;

  // Used for actually moving the motor to a given position with PID applied to a voltage input.
  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOSim() {}

  @Override
  public void setMotor(TalonFX motor) {
    this.motorSim = motor.getSimState();
    this.motor = motor;
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
    updateSim();

    inputs.leftCarriagePositionInches = Meters.of(innerClimbSim.getPositionMeters()).in(Inches);
    inputs.leftMotorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.leftMotorRotations = motor.getPosition().getValueAsDouble();
    inputs.leftMotorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.leftMotorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();

    inputs.rightCarriagePositionInches = Meters.of(outerClimbSim.getPositionMeters()).in(Inches);
    inputs.rightMotorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.rightMotorRotations = motor.getPosition().getValueAsDouble();
    inputs.rightMotorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.rightMotorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();
  }

  private void updateSim() {
    motorSim.setSupplyVoltage(Volts.of(12));
    double motorInverted = -1.0; // -1 for inverted, 1 for forward motor.

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    innerClimbSim.setInputVoltage(motorInverted * motorSim.getMotorVoltage());
    innerClimbSim.update(0.02); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", motorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", innerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", innerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", innerClimbSim.hasHitLowerLimit());
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", motorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());

    motorSim.setRawRotorPosition(
        motorInverted * getMotorRotations(innerClimbSim.getPositionMeters()));

    // angular velocity = linear velocity / radius, taken also from 5414
    motorSim.setRotorVelocity(
        motorInverted
            * ((innerClimbSim.getVelocityMetersPerSecond()
                    / ClimbConstants.INNERCLIMB_SPOOL_RADIUS.in(Meters))
                // radians/sec to rotations/sec
                / (2.0 * Math.PI))
            * ClimbConstants.LEFTMOTOR_TO_CLIMB_GEARING);
  }

  @Override
  public void setMotorSetpoint(Angle setpoint) {
    motor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Volts));
  }

  @Override
  public Angle getMotorPosition() {
    return motor.getPosition().getValue();
  }

  /**
   * @return The height of the first stage of the elevator.
   */
  @Override
  public Distance getCarriagePosition() {
    return Meters.of(innerClimbSim.getPositionMeters());
  }

  /**
   * Source: 5414 Pearadox Converts the elevators position (meters) to motor rotations based on the
   * elevator spool radius and motor gearing.
   *
   * @param elevatorPosition
   * @return
   */
  private static double getMotorRotations(double elevatorPosition) {
    // angular displacement in radians = linear displacement / radius
    return Units.radiansToRotations(
            elevatorPosition / ClimbConstants.INNERCLIMB_SPOOL_RADIUS.in(Meters))
        // multiply by gear ratio
        * ClimbConstants.LEFTMOTOR_TO_CLIMB_GEARING;
  }
}
