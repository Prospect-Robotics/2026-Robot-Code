package com.team2813.subsystems.intakeextension;

import static com.team2813.subsystems.intakeextension.IntakeExtensionConstants.toIntakeExtensionPosition;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Code that controls the extension of the intake and the front plate of the hopper via a rack and
 * pinion gear.
 */
public class IntakeExtension extends SubsystemBase implements AutoCloseable {
  private final TalonFX extenderMotor;
  private final IntakeExtensionIO io;
  private final IntakeExtensionIOInputsAutoLogged replayedInputs =
      new IntakeExtensionIOInputsAutoLogged();

  private Angle extensionSetpoint = Rotation.of(0);
  private boolean extenderAtPosition = true;
  private PositionVoltage extensionSetpointPositionVoltage = new PositionVoltage(extensionSetpoint);

  public IntakeExtension(IntakeExtensionIO io) {
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    extenderMotor.getConfigurator().apply(IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG);
    extenderMotor.setPosition(Rotation.of(0)); // intake should be fully retracted on bootup
    this.io = io;
    this.io.setMotor(extenderMotor);

    // Start with the intake retracted.
    resetSetpoint(IntakeExtensionConstants.ExtenderPositions.IN);
  }

  @Override
  public void periodic() {
    // Update the IO inputs from the real/simulated hardware.
    io.updateState(replayedInputs);
    // Update the subsystem state based on the new IO inputs.
    replayedInputs.extenderMotorVoltage = extenderMotor.getMotorVoltage().getValue();
    replayedInputs.extenderMotorRPS = extenderMotor.getRotorVelocity().getValue();
    replayedInputs.extenderMotorCurrent = extenderMotor.getStatorCurrent().getValue();
    replayedInputs.extenderMotorPosition = extenderMotor.getPosition().getValue();
    replayedInputs.extenderMotorSetpoint = extensionSetpoint;

    double error =
        replayedInputs
            .extenderMotorPosition
            .minus(replayedInputs.extenderMotorSetpoint)
            .abs(Rotation);

    // Is the error between the setpoint greater than half a rotation.
    extenderAtPosition = error <= 0.4;

    Logger.recordOutput("IntakeExtension/extenderAtPosition", extenderAtPosition);
    Logger.processInputs("IntakeExtension", replayedInputs);
  }

  @Override
  public void simulationPeriodic() {
    SimulationVisualizer.getInstance()
        .updateIntakeExtensionPosition(
            toIntakeExtensionPosition(replayedInputs.extenderMotorPosition));
  }

  public boolean isExtenderAtPosition() {
    return extenderAtPosition;
  }

  /** Resets the motor setpoint and update related subsystem state variables. */
  private void resetSetpoint(IntakeExtensionConstants.ExtenderPositions position) {
    extensionSetpoint = IntakeExtensionConstants.toMotorSetpoint(position);
    extensionSetpointPositionVoltage.withPosition(extensionSetpoint);
    extenderMotor.setControl(extensionSetpointPositionVoltage);
  }

  public void extend() {
    resetSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT);
    extenderAtPosition = false;
  }

  public void retract() {
    resetSetpoint(IntakeExtensionConstants.ExtenderPositions.IN);
    extenderAtPosition = false;
  }

  public void setExtenderVoltage(Voltage extensionVoltage) {
    extenderMotor.setVoltage(extensionVoltage.in(Volts));
  }

  public void stopMotor() {
    extenderMotor.setVoltage(0);
  }

  public Angle getSetpoint() {
    return replayedInputs.extenderMotorSetpoint;
  }

  public Angle getPosition() {
    return replayedInputs.extenderMotorPosition;
  }

  @Override
  public void close() {
    extenderMotor.close();
  }
}
