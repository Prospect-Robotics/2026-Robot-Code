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

public class ClimbIOSim extends ClimbIO {
  public static final Mass APPROX_CLIMB_CARRIAGE_WEIGHT = Pounds.of(2);

  private final ElevatorSim climbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          climbConstants.motorToClimbGearing(),
          APPROX_CLIMB_CARRIAGE_WEIGHT.in(Kilograms),
          climbConstants.climbSpoolRadius().in(Meters),
          climbConstants.climbMinHeight().in(Meters),
          climbConstants.climbMaxHeight().in(Meters),
          true,
          climbConstants.climbMinHeight().in(Meters));

  private TalonFX climbMotor;
  private TalonFXSimState climbMotorSim;
  // Used for actually moving the motor to a given position with PID applied to a voltage input.
  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOSim(AllClimbConstants climbConstants) {
    super(climbConstants);

    climbMotor = new TalonFX(climbConstants.climbCanID());
    climbMotor.getConfigurator().apply(climbConstants.climbMotorConfig());
    climbMotorSim = climbMotor.getSimState();
    climbMotorSim.Orientation = getOrientation(climbMotor);
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
    updateSim();

    inputs.carriagePositionInches = Meters.of(climbSim.getPositionMeters()).in(Inches);
    inputs.motorCurrent = climbMotor.getStatorCurrent().getValueAsDouble();
    inputs.motorRotations = climbMotor.getPosition().getValueAsDouble();
    inputs.motorVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorVelocityRotsPerSecond = climbMotor.getRotorVelocity().getValue();
  }

  private void updateSim() {
    climbMotorSim.setSupplyVoltage(Volts.of(12));

    // Apply the voltage to the sim elevator that we apply to the sim motor.
    // Negating the sim motor value since it is set to use negative value when pushing
    // the cartrage UP.
    climbSim.setInput(climbMotorSim.getMotorVoltage());
    climbSim.update(Constants.SIM_TIME_PERIOD); // Same update cycle as an actual robot, 20 ms.

    // Logs to "Real Outputs" NT
    Logger.recordOutput(
        String.format("Simulated Climb/%s/motorSim/Voltage", climbConstants.climbName()),
        climbMotorSim.getMotorVoltage());
    Logger.recordOutput(
        String.format(
            "Simulated Climb/%s/climbSim/position (meters)", climbConstants.climbName()),
        climbSim.getPositionMeters());
    Logger.recordOutput(
        String.format(
            "Simulated Climb/%s/climbSim/hitsUpperLimit", climbConstants.climbName()),
        climbSim.hasHitUpperLimit());
    Logger.recordOutput(
        String.format(
            "Simulated Climb/%s/climbSim/hitsLowerLimit", climbConstants.climbName()),
        climbSim.hasHitLowerLimit());

    // angular velocity = linear velocity / radius, taken also from 5414
    climbMotorSim.setRotorVelocity(
        climbSim.getVelocityMetersPerSecond()
            / ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Meters));

    climbMotorSim.setRawRotorPosition(getMotorRotations(climbSim.getPositionMeters()));
  }

  @Override
  public void setMotorSetpoint(Angle setpoint) {
    climbMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void stopMotor() {
    climbMotor.disable();
  }

  @Override
  public Angle getMotorPosition() {
    return climbMotor.getPosition().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage motorVoltage) {
    climbMotor.setVoltage(motorVoltage.in(Volts));
  }

  /**
   * @return The height of the climb.
   */
  @Override
  public Distance getCarriagePosition() {
    return Meters.of(climbSim.getPositionMeters());
  }

  /**
   * Source: 5414 Pearadox Converts the elevators position (meters) to motor rotations based on the
   * elevator spool radius and motor gearing.
   *
   * @param elevatorPosition
   * @return
   */
  private double getMotorRotations(double elevatorPosition) {
    // angular displacement in radians = linear displacement / radius
    return elevatorPosition / climbConstants.climbHeightChangePerRotation().in(Meters);
  }

  /**
   * Checks the set orientation of a given motor and updates the sim state to match.
   *
   * @param motor The {@link TalonFX} instance to get the inversion of.
   * @return A {@link ChassisReference} for the {@link TalonFXSimState} to consume.
   */
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
