package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim implements ClimbIO {
  public static final Mass APPROX_CLIMB_CARRIAGE_WEIGHT = Pounds.of(2);

  // Physics sim for the climb
  // TODO(stefan): Factor out the two climbs in a helper class. There's a lot of
  // repetition here and in the class methods below.
  private final ElevatorSim innerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.INNER_MOTOR_TO_CLIMB_GEARING,
          APPROX_CLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          ClimbConstants.INNER_CLIMB_SPOOL_RADIUS.in(Meter),
          ClimbConstants.INNER_CLIMB_MIN_HEIGHT.in(Meter),
          ClimbConstants.INNER_CLIMB_MAX_HEIGHT.in(Meter),
          true,
          ClimbConstants.INNER_CLIMB_MIN_HEIGHT.in(Meter));

  private final ElevatorSim outerClimbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.OUTER_MOTOR_TO_CLIMB_GEARING,
          APPROX_CLIMB_CARRIAGE_WEIGHT.in(Kilograms),
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

  public ClimbIOSim() {
    innerMotor = new TalonFX(Constants.LEFTCLIMB_MOTOR_ID);
    innerMotor.getConfigurator().apply(ClimbConstants.INNER_MOTOR_TO_CLIMB_CONFIG);
    innerMotorSim = innerMotor.getSimState();
    innerMotorSim.Orientation = getOrientation(innerMotor);

    outerMotor = new TalonFX(Constants.RIGHTCLIMB_MOTOR_ID);
    outerMotor.getConfigurator().apply(ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG);
    outerMotorSim = outerMotor.getSimState();
    outerMotorSim.Orientation = getOrientation(outerMotor);
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
    updateSim();

    inputs.innerCarriagePositionInches = Meters.of(innerClimbSim.getPositionMeters()).in(Inches);
    inputs.innerMotorCurrent = innerMotor.getStatorCurrent().getValueAsDouble();
    inputs.innerMotorRotations = innerMotor.getPosition().getValueAsDouble();
    inputs.innerMotorVoltage = innerMotor.getMotorVoltage().getValueAsDouble();
    inputs.innerMotorVelocityRotsPerSecond = innerMotor.getRotorVelocity().getValue();

    inputs.outerCarriagePositionInches = Meters.of(outerClimbSim.getPositionMeters()).in(Inches);
    inputs.outerMotorCurrent = outerMotor.getStatorCurrent().getValueAsDouble();
    inputs.outerMotorRotations = outerMotor.getPosition().getValueAsDouble();
    inputs.outerMotorVoltage = outerMotor.getMotorVoltage().getValueAsDouble();
    inputs.outerMotorVelocityRotsPerSecond = outerMotor.getRotorVelocity().getValue();
  }

  private void updateSim() {
    innerMotorSim.setSupplyVoltage(Volts.of(12));

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    innerClimbSim.setInput(innerMotorSim.getMotorVoltage());
    innerClimbSim.update(Constants.SIM_TIME_PERIOD); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput("Simulated Climb/Inner/motorSim/Voltage", innerMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/Inner/climbSim/position (meters)", innerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/Inner/climbSim/hitsUpperLimit", innerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/Inner/climbSim/hitsLowerLimit", innerClimbSim.hasHitLowerLimit());

    innerMotorSim.setRawRotorPosition(getLeftMotorRotations(innerClimbSim.getPositionMeters()));

    // angular velocity = linear velocity / radius, taken also from 5414
    innerMotorSim.setRotorVelocity(
        innerClimbSim.getVelocityMetersPerSecond()
            / ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Meters));

    outerMotorSim.setSupplyVoltage(Volts.of(12));

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    outerClimbSim.setInput(outerMotorSim.getMotorVoltage());
    outerClimbSim.update(Constants.SIM_TIME_PERIOD); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput("Simulated Climb/Outer/motorSim/Voltage", outerMotorSim.getMotorVoltage());
    Logger.recordOutput(
        "Simulated Climb/Outer/climbSim/position (meters)", outerClimbSim.getPositionMeters());
    Logger.recordOutput(
        "Simulated Climb/Outer/climbSim/hitsUpperLimit", outerClimbSim.hasHitUpperLimit());
    Logger.recordOutput(
        "Simulated Climb/Outer/climbSim/hitsLowerLimit", outerClimbSim.hasHitLowerLimit());

    // angular velocity = linear velocity / radius, taken also from 5414
    outerMotorSim.setRotorVelocity(
        outerClimbSim.getVelocityMetersPerSecond()
            / ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Meters));

    outerMotorSim.setRawRotorPosition(getRightMotorRotations(outerClimbSim.getPositionMeters()));
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

  @Override
  public void setInnerMotorVoltage(Voltage motorVoltage) {
    innerMotor.setVoltage(motorVoltage.in(Volts));
  }

  @Override
  public void setOuterMotorVoltage(Voltage motorVoltage) {
    innerMotor.setVoltage(motorVoltage.in(Volts));
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
    return Meters.of(outerClimbSim.getPositionMeters());
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
    return elevatorPosition
        / ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Meters);
  }

  private static double getRightMotorRotations(double elevatorPosition) {
    // angular displacement in radians = linear displacement / radius
    return elevatorPosition
        / ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Meters);
  }

  private static ChassisReference getOrientation(TalonFX motor) {
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    // Populate "outputConfigs" with the current configuration of the motor.
    motor.getConfigurator().refresh(outputConfigs);

    return switch (outputConfigs.Inverted) {
      case Clockwise_Positive -> ChassisReference.Clockwise_Positive;
      case CounterClockwise_Positive -> ChassisReference.CounterClockwise_Positive;
    };
  }
}
