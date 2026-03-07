package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.subsystems.Simulation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public final class IntakeExtensionIOSim implements IntakeExtensionIO {
  private final IntakeExtensionIOReal realIntakeExtension;
  private final ElevatorSim extenderSim;
  // PID controller for simulation - TalonFX sim doesn't run internal PID
  private final PIDController simPidController;
  private boolean pidControlEnabled;

  private final TalonFXSimState extenderMotorSimState;

  public IntakeExtensionIOSim() {
    realIntakeExtension = new IntakeExtensionIOReal();
    extenderMotorSimState = realIntakeExtension.getExtenderMotorSimState();

    double minHeightMeters = IntakeExtensionConstants.RETRACTED_POSITION.in(Meters);
    double maxHeightMeters = IntakeExtensionConstants.EXTENDED_POSITION.in(Meters);
    extenderSim =
        new ElevatorSim(
            DCMotor.getKrakenX44(1),
            /* gearing= */ IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING,
            /* carriageMassKg= */ IntakeExtensionConstants.WEIGHT_OF_EXTENDER_CARRIAGE.in(
                Kilograms),
            /* drumRadiusMeters= */ IntakeExtensionConstants.PULLEY_RADIUS.in(Meters),
            minHeightMeters,
            maxHeightMeters,
            /* simulateGravity= */ false,
            /* startingHeightMeters= */ minHeightMeters);

    // Initialize PID controller with same gains as motor config
    var slot0 = IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.Slot0;
    simPidController = new PIDController(slot0.kP, slot0.kI, slot0.kD);
  }

  @Override
  public void updateState(IntakeExtensionIOInputs inputs) {
    // Continue supplying the simulated motor with 12V voltage.
    extenderMotorSimState.setSupplyVoltage(Simulation.getMotorSupplyVoltage());

    if (pidControlEnabled) {
      // TalonFX simulation doesn't run the internal PID controller, so we simulate it ourselves
      // using a WPILib PIDController with the same gains.
      double currentPositionRotations =
          extenderSim.getPositionMeters()
              * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS;
      double setpointRotations = simPidController.getSetpoint();

      // Calculate PID output voltage
      double pidOutput = simPidController.calculate(currentPositionRotations, setpointRotations);

      // Add static friction feedforward (kS). Velocity feedforward (kV) is not used in this
      // position-control simulation.
      var slot0 = IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.Slot0;
      double feedforward = Math.signum(pidOutput) * slot0.kS;

      // Clamp to supply voltage
      double supplyVoltage = Simulation.getMotorSupplyVoltage().in(Volts);
      double motorVoltage = MathUtil.clamp(pidOutput + feedforward, -supplyVoltage, supplyVoltage);

      extenderSim.setInput(motorVoltage);
    }

    // With the new motor voltage, step forward the simulation by 20ms. This will update the
    // position and velocity of the simulated mechanism.
    extenderSim.update(0.02);

    // Now the simulated physical movement of the mechanism is fed back into the TalonFX motor
    // simulation so its internal state and sensor readings (position/velocity) stay consistent with
    // the ElevatorSim physics. PID control is handled by simPidController, not by the TalonFX.
    // We apply MOTOR_DIRECTION because when the motor is inverted, getPosition() will negate the
    // raw rotor position. By setting the raw rotor position with the opposite sign, getPosition()
    // will return the correct positive value for extended positions.
    extenderMotorSimState.setRotorVelocity(
        extenderSim.getVelocityMetersPerSecond()
            * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);
    extenderMotorSimState.setRawRotorPosition(
        extenderSim.getPositionMeters()
            * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);

    // Finally update all simulated inputs.
    realIntakeExtension.updateState(inputs);
    inputs.extenderMotorPosition =
        Rotations.of(
            extenderSim.getPositionMeters()
                * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);
  }

  @Override
  public void setExtensionSetpoint(Angle setpoint) {
    pidControlEnabled = true;
    realIntakeExtension.setExtensionSetpoint(setpoint);
    simPidController.setSetpoint(setpoint.in(Rotations));
  }

  @Override
  public void close() {
    realIntakeExtension.close();
    simPidController.close();
  }

  @Override
  public void setExtenderVoltage(Voltage extensionVoltage) {
    pidControlEnabled = false;
    realIntakeExtension.setExtenderVoltage(extensionVoltage);
    extenderSim.setInputVoltage(extensionVoltage.in(Volts));
  }
}
