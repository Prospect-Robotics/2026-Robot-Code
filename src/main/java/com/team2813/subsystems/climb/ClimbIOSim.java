package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim implements ClimbIO {

  // Physics sim for the climb
  private final ElevatorSim innerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING,
          ClimbConstants.INNER_CLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          ClimbConstants.INNER_CLIMB_SPOOL_RADIUS.in(Meter),
          ClimbConstants.INNER_CLIMB_MIN_HEIGHT.in(Meter),
          ClimbConstants.INNER_CLIMB_MAX_HEIGHT.in(Meter),
          true,
          ClimbConstants.INNER_CLIMB_MIN_HEIGHT.in(Meter));

  private final ElevatorSim outerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.OUTER_MOTOR_TO_CLIMB_GEARING,
          ClimbConstants.OUTER_CLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS.in(Meter),
          ClimbConstants.OUTER_CLIMB_MIN_HEIGHT.in(Meter),
          ClimbConstants.OUTER_CLIMB_MAX_HEIGHT.in(Meter),
          true,
          ClimbConstants.OUTER_CLIMB_MIN_HEIGHT.in(Meter));

  private TalonFX innerMotor;
  private TalonFXSimState innerMotorSim;
  private TalonFX outerMotor;
  private TalonFXSimState outerMotorSim;
  // Used for actually moving the motor to a given position with PID applied to a voltage input.
  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  // -1 for inverted, 1 for forward motor. Used because the sim is inverted comapared to real life.
  private final double MOTOR_INVERTED = -1.0;

  public ClimbIOSim() {
    innerMotor = new TalonFX(0);
    innerMotor.getConfigurator().apply(ClimbConstants.INNER_MOTOR_TO_CLIMB_CONFIG);
    innerMotorSim = innerMotor.getSimState();
    outerMotor = new TalonFX(0);
    outerMotor.getConfigurator().apply(ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG);
    outerMotorSim = outerMotor.getSimState();
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
    updateSim();

    inputs.innerCarriagePositionInches = Meters.of(innerClimbSim.getPositionMeters()).in(Inches);
    inputs.innerMotorCurrent = innerMotor.getStatorCurrent().getValueAsDouble();
    inputs.innerMotorRotations = innerMotor.getPosition().getValueAsDouble();
    inputs.innerMotorVoltage = innerMotor.getMotorVoltage().getValueAsDouble();
    inputs.innerMotorVelocityRotsPerSecond = innerMotor.getVelocity().getValueAsDouble();

    inputs.outerCarriagePositionInches = Meters.of(outerClimbSim.getPositionMeters()).in(Inches);
    inputs.outerMotorCurrent = outerMotor.getStatorCurrent().getValueAsDouble();
    inputs.outerMotorRotations = outerMotor.getPosition().getValueAsDouble();
    inputs.outerMotorVoltage = outerMotor.getMotorVoltage().getValueAsDouble();
    inputs.outerMotorVelocityRotsPerSecond = outerMotor.getVelocity().getValueAsDouble();
  }

  private void updateSim() {
    innerMotorSim.setSupplyVoltage(Volts.of(12));

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    innerClimbSim.setInputVoltage(MOTOR_INVERTED * innerMotorSim.getMotorVoltage());
    innerClimbSim.update(Constants.SIM_TIME_PERIOD); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", innerMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", innerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", innerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", innerClimbSim.hasHitLowerLimit());
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", innerMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());

    innerMotorSim.setRawRotorPosition(
        MOTOR_INVERTED * getLeftMotorRotations(innerClimbSim.getPositionMeters()));

    // angular velocity = linear velocity / radius, taken also from 5414
    innerMotorSim.setRotorVelocity(
        MOTOR_INVERTED
            * ((innerClimbSim.getVelocityMetersPerSecond()
                    / ClimbConstants.INNER_CLIMB_SPOOL_RADIUS.in(Meters))
                // radians/sec to rotations/sec
                / (2.0 * Math.PI))
            * ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING);

    outerMotorSim.setSupplyVoltage(Volts.of(12));

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    outerClimbSim.setInputVoltage(MOTOR_INVERTED * outerMotorSim.getMotorVoltage());
    outerClimbSim.update(0.02); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", outerMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());
    Logger.recordOutput("Simulated Climb/motorSim/Voltage", outerMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());

    innerMotorSim.setRawRotorPosition(
        MOTOR_INVERTED * getRightMotorRotations(outerClimbSim.getPositionMeters()));

    // angular velocity = linear velocity / radius, taken also from 5414
    innerMotorSim.setRotorVelocity(
        MOTOR_INVERTED
            * ((innerClimbSim.getVelocityMetersPerSecond()
                    / ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS.in(Meters))
                // radians/sec to rotations/sec
                / (2.0 * Math.PI))
            * ClimbConstants.OUTER_MOTOR_TO_CLIMB_GEARING);
  }

  @Override
  public void setInnerMotorSetpoint(Angle setpoint) {
    innerMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setOuterMotorSetpoint(Angle setpoint) {
    outerMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void stopInnerMotor() {
    innerMotor.disable();
  }

  @Override
  public void stopOuterMotor() {
    outerMotor.disable();
  }

  @Override
  public Angle getInnerMotorPosition() {
    return innerMotor.getPosition().getValue();
  }

  @Override
  public Angle getOuterMotorPosition() {
    return outerMotor.getPosition().getValue();
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
        * ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING;
  }

  private static double getRightMotorRotations(double elevatorPosition) {
    // angular displacement in radians = linear displacement / radius
    return Units.radiansToRotations(
            elevatorPosition / ClimbConstants.OUTER_CLIMB_SPOOL_RADIUS.in(Meters))
        // multiply by gear ratio
        * ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING;
  }
}
