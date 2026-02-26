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

  // Physics sim for the climb
  private final ElevatorSim innerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.LEFT_MOTOR_TO_CLIMB_GEARING,
          ClimbConstants.INNER_CLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          ClimbConstants.INNER_CLIMB_SPOOL_RADIUS.in(Meter),
          ClimbConstants.INNER_CLIMB_MIN_HEIGHT.in(Meter),
          ClimbConstants.INNER_CLIMB_MAX_HEIGHT.in(Meter),
          true,
          ClimbConstants.INNER_CLIMB_MIN_HEIGHT.in(Meter));

  private final ElevatorSim outerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.RIGHT_MOTOR_TO_CLIMB_GEARING,
          ClimbConstants.OUTER_CLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS.in(Meter),
          ClimbConstants.OUTER_CLIMB_MIN_HEIGHT.in(Meter),
          ClimbConstants.OUTER_CLIMB_MAX_HEIGHT.in(Meter),
          true,
          ClimbConstants.OUTER_CLIMB_MIN_HEIGHT.in(Meter));

  private TalonFX leftMotor;
  private TalonFXSimState leftMotorSim;
  private TalonFX rightMotor;
  private TalonFXSimState rightMotorSim;
  // Used for actually moving the motor to a given position with PID applied to a voltage input.
  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOSim() {
    leftMotor = new TalonFX(0);
    leftMotor.getConfigurator().apply(ClimbConstants.LEFT_MOTOR_TO_CLIMB_CONFIG);
    leftMotorSim = leftMotor.getSimState();
    rightMotor = new TalonFX(0);
    rightMotor.getConfigurator().apply(ClimbConstants.RIGHT_MOTOR_TO_CLIMB_CONFIG);
    rightMotorSim = rightMotor.getSimState();
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
    updateSim();

    inputs.innerCarriagePositionInches = Meters.of(innerClimbSim.getPositionMeters()).in(Inches);
    inputs.innerMotorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.innerMotorRotations = leftMotor.getPosition().getValueAsDouble();
    inputs.innerMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.innerMotorVelocityRotsPerSecond = leftMotor.getVelocity().getValueAsDouble();

    inputs.outerCarriagePositionInches = Meters.of(outerClimbSim.getPositionMeters()).in(Inches);
    inputs.outerMotorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.outerMotorRotations = rightMotor.getPosition().getValueAsDouble();
    inputs.outerMotorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.outerMotorVelocityRotsPerSecond = rightMotor.getVelocity().getValueAsDouble();
  }

  private void updateSim() {
    leftMotorSim.setSupplyVoltage(Volts.of(12));
    double motorInverted = -1.0; // -1 for inverted, 1 for forward motor.

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    innerClimbSim.setInputVoltage(motorInverted * leftMotorSim.getMotorVoltage());
    innerClimbSim.update(0.02); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", leftMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", innerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", innerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", innerClimbSim.hasHitLowerLimit());
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", leftMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());

    leftMotorSim.setRawRotorPosition(
        motorInverted * getLeftMotorRotations(innerClimbSim.getPositionMeters()));

    // angular velocity = linear velocity / radius, taken also from 5414
    leftMotorSim.setRotorVelocity(
        motorInverted
            * ((innerClimbSim.getVelocityMetersPerSecond()
                    / ClimbConstants.INNER_CLIMB_SPOOL_RADIUS.in(Meters))
                // radians/sec to rotations/sec
                / (2.0 * Math.PI))
            * ClimbConstants.LEFT_MOTOR_TO_CLIMB_GEARING);

    rightMotorSim.setSupplyVoltage(Volts.of(12));

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    outerClimbSim.setInputVoltage(motorInverted * rightMotorSim.getMotorVoltage());
    outerClimbSim.update(0.02); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", rightMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", rightMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());

    leftMotorSim.setRawRotorPosition(
        motorInverted * getRightMotorRotations(outerClimbSim.getPositionMeters()));

    // angular velocity = linear velocity / radius, taken also from 5414
    leftMotorSim.setRotorVelocity(
        motorInverted
            * ((innerClimbSim.getVelocityMetersPerSecond()
                    / ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS.in(Meters))
                // radians/sec to rotations/sec
                / (2.0 * Math.PI))
            * ClimbConstants.RIGHT_MOTOR_TO_CLIMB_GEARING);
  }

  @Override
  public void setInnerMotorSetpoint(Angle setpoint) {
    leftMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setOuterMotorSetpoint(Angle setpoint) {
    rightMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setInnerMotorVoltage(Voltage voltage) {
    leftMotor.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setOuterMotorVoltage(Voltage voltage) {
    rightMotor.setVoltage(voltage.in(Volts));
  }

  @Override
  public Angle getInnerMotorPosition() {
    return leftMotor.getPosition().getValue();
  }

  @Override
  public Angle getOuterMotorPosition() {
    return rightMotor.getPosition().getValue();
  }

  /**
   * @return The height of the climb.
   */
  @Override
  public Distance getInnerCarriagePosition() {
    return Meters.of(innerClimbSim.getPositionMeters());
  }

  @Override
  public Distance getOuterCarriagePosition() {
    return Meters.of(innerClimbSim.getPositionMeters());
  }

  /**
   * Source: 5414 Pearadox Converts the elevators position (meters) to motor rotations based on the
   * elevator spool radius and motor gearing.
   *
   * @param elevatorPosition
   * @return
   */
  private static double getLeftMotorRotations(double elevatorPosition) {
    // angular displacement in radians = linear displacement / radius
    return Units.radiansToRotations(
            elevatorPosition / ClimbConstants.INNER_CLIMB_SPOOL_RADIUS.in(Meters))
        // multiply by gear ratio
        * ClimbConstants.LEFT_MOTOR_TO_CLIMB_GEARING;
  }

  private static double getRightMotorRotations(double elevatorPosition) {
    // angular displacement in radians = linear displacement / radius
    return Units.radiansToRotations(
            elevatorPosition / ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS.in(Meters))
        // multiply by gear ratio
        * ClimbConstants.LEFT_MOTOR_TO_CLIMB_GEARING;
  }
}
