package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeExtensionIOSim implements IntakeExtensionIO {
  private final TalonFX extenderMotor;
  private final TalonFXSimState extenderMotorSimState;

  private final Resistance motorResistance = MilliOhm.of(2);

  private final DCMotorSim extenderSim;

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
        //        new ElevatorSim(
        //            DCMotor.getKrakenX44(2),
        //            IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING,
        //            IntakeExtensionConstants.WEIGHT_OF_EXTENDER_CARRIAGE.in(Kilograms),
        //            IntakeExtensionConstants.PULLEY_RADIUS.in(Meters),
        //            IntakeExtensionConstants.RETRACTED_POSITION.in(Meters),
        //            IntakeExtensionConstants.EXTENDED_POSITION.in(Meters),
        //            false,
        //            0); // Start unextended
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                IntakeExtensionConstants.MOTOR_MOMENT_OF_INERTIA,
                IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING),
            DCMotor.getKrakenX60(1));

    // Initialize PID controller with same gains as motor config
    var slot0 = IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.Slot0;
    simPidController = new PIDController(slot0.kP, slot0.kI, slot0.kD);

    extensionSetpoint = Rotation.of(0);
  }

  @Override
  public void updateState(IntakeExtensionIOInputs inputs) {
    extenderSim.setInputVoltage(extenderMotorSimState.getMotorVoltage());

    extenderSim.update(Constants.SIM_TIME_PERIOD);

    // Try with final before without
    final double pos_rot = extenderSim.getAngularPositionRotations();
    final double vel_rps = Units.radiansToRotations(extenderSim.getAngularVelocityRadPerSec());

    extenderMotorSimState.setRawRotorPosition(pos_rot);
    extenderMotorSimState.setRotorVelocity(vel_rps);

    extenderMotorSimState.setSupplyVoltage(
        12 - extenderMotorSimState.getSupplyCurrent() * motorResistance.in(Ohm));
  }

  @Override
  public void setExtensionSetpoint(Angle setpoint) {
    extensionSetpoint = setpoint;
    // In simulation we do not call setControl; PID is handled manually by simPidController using
    // extensionSetpoint
  }

  @Override
  public void close() {
    extenderMotor.close();
  }

  @Override
  public void setExtenderVoltage(Voltage extensionVoltage) {
    extenderMotor.setVoltage(extensionVoltage.in(Volts));
    extenderSim.setInputVoltage(extensionVoltage.in(Volts));
  }
}
