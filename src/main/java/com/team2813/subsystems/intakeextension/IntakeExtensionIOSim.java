package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class IntakeExtensionIOSim implements IntakeExtensionIO {
  private final TalonFX extenderMotor;
  private final TalonFXSimState extenderMotorSimState;

  private final ElevatorSim extenderSim;

  // PID controller for simulation - TalonFX sim doesn't run internal PID
  private final PIDController simPidController;

  private Angle extensionSetpoint;

  // Motor to extension direction depends on how the motor is geared with respect to the extension
  // mechanism.
  private static final double MOTOR_DIRECTION =
      IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.MotorOutput.Inverted
              == InvertedValue.CounterClockwise_Positive
          ? 1.0
          : -1.0;

  public IntakeExtensionIOSim() {
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    extenderMotorSimState = extenderMotor.getSimState();

    extenderMotor.getConfigurator().apply(IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG);

    extenderSim =
        new ElevatorSim(
            DCMotor.getKrakenX44(1),
            IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING,
            IntakeExtensionConstants.WEIGHT_OF_EXTENDER_CARRIAGE.in(Kilograms),
            IntakeExtensionConstants.PULLEY_RADIUS.in(Meters),
            IntakeExtensionConstants.RETRACTED_POSITION.in(Meters),
            IntakeExtensionConstants.EXTENDED_POSITION.in(Meters),
            false,
            0); // Start unextended

    // Initialize PID controller with same gains as motor config
    var slot0 = IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.Slot0;
    simPidController = new PIDController(slot0.kP, slot0.kI, slot0.kD);

    extensionSetpoint = Rotation.of(0);
  }

  @Override
  public void updateState(IntakeExtensionIOInputs inputs) {
    // Continue supplying the simulated motor with 12V voltage.
    extenderMotorSimState.setSupplyVoltage(12);

    // TalonFX simulation doesn't run the internal PID controller, so we simulate it ourselves
    // using a WPILib PIDController with the same gains.
    double currentPositionRotations =
        extenderSim.getPositionMeters()
            * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS;
    double setpointRotations = extensionSetpoint.in(Rotations);

    // Calculate PID output voltage
    double pidOutput = simPidController.calculate(currentPositionRotations, setpointRotations);

    // Add feedforward (kS for static friction, kV for velocity)
    var slot0 = IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.Slot0;
    double feedforward = Math.signum(pidOutput) * slot0.kS;

    // Clamp to supply voltage
    double motorVoltage = MathUtil.clamp(pidOutput + feedforward, -12.0, 12.0);

    extenderSim.setInput(motorVoltage);

    // With the new motor voltage, step forward the simulation by 20ms. This will update the
    // position and velocity of the simulated mechanism.
    extenderSim.update(0.02);

    // Now the simulated physical movement of the mechanism is fed back into the motor simulation so
    // that its simulated PID controller can give us new simulated motor voltage next time around.
    // We apply MOTOR_DIRECTION because when the motor is inverted, getPosition() will negate the
    // raw rotor position. By setting the raw rotor position with the opposite sign, getPosition()
    // will return the correct positive value for extended positions.
    extenderMotorSimState.setRotorVelocity(
        MOTOR_DIRECTION
            * extenderSim.getVelocityMetersPerSecond()
            * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);
    extenderMotorSimState.setRawRotorPosition(
        MOTOR_DIRECTION
            * extenderSim.getPositionMeters()
            * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);

    // Finally update all simulated inputs.
    // Report position directly from ElevatorSim to avoid TalonFX inversion issues
    inputs.extenderMotorVoltage = extenderMotor.getMotorVoltage().getValue();
    inputs.extenderMotorRPS = extenderMotor.getRotorVelocity().getValue();
    inputs.extenderMotorCurrent = extenderMotor.getStatorCurrent().getValue();
    inputs.extenderMotorPosition =
        Rotations.of(
            extenderSim.getPositionMeters()
                * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);
    inputs.extenderMotorSetpoint = extensionSetpoint;
  }

  @Override
  public void setExtensionSetpoint(Angle setpoint) {
    extensionSetpoint = setpoint;
    // Note: setControl is still called for compatibility, but we simulate PID ourselves
  }

  @Override
  public void close() {
    extenderMotor.close();
  }
}
