package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class IntakeExtensionIOSim implements IntakeExtensionIO {
  TalonFX extenderMotor;
  private TalonFXSimState extenderMotorSimState;
  private ElevatorSim extenderSim;

  // Motor to extension direction depends on how the motor is geared with respect to the extension
  // mechanism.
  private static final double MOTOR_DIRECTION = 1.0;
  // IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.MotorOutput.Inverted
  //         == InvertedValue.CounterClockwise_Positive
  //     ? 1.0
  //     : -1.0;

  public IntakeExtensionIOSim() {
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
  }

  @Override
  public void setMotor(TalonFX motor) {
    extenderMotor = motor;
    extenderMotorSimState = motor.getSimState();
    extenderMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
    extenderMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
  }

  @Override
  public void updateState(IntakeExtensionIOInputs inputs) {
    extenderMotorSimState = extenderMotor.getSimState();
    // Continue supplying the simulated motor with 12V voltage.
    extenderMotorSimState.setSupplyVoltage(12);

    // Read back the actual motor control voltage as calculated by the extenderMotorSimState,
    // which internally takes the setpoint and position error + PID Values into account.
    extenderSim.setInput(MOTOR_DIRECTION * extenderMotorSimState.getMotorVoltage());

    // With the new motor voltage, step forward the simulation by 20ms. This will update the
    // position and velocity of the simulated mechanism.
    extenderSim.update(0.02);

    // No the simulated physical movement of the mechanism is fed back into the motor simulation so
    // that its simulated PID controler can give us new simulated motor voltage next time around.
    double extenderSimPositionsInM = extenderSim.getPositionMeters();
    extenderMotorSimState.setRotorVelocity(
        MOTOR_DIRECTION
            * extenderSim.getVelocityMetersPerSecond()
            * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);
    extenderMotorSimState.setRawRotorPosition(
        MOTOR_DIRECTION
            * extenderSimPositionsInM
            * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);
  }
}
