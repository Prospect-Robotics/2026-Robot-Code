package com.team2813.subsystems.intakeextension;

import static com.team2813.subsystems.intakeextension.IntakeExtensionConstants.toIntakeExtensionPosition;
import static edu.wpi.first.units.Units.*;

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
public class IntakeExtension extends SubsystemBase {
  private final TalonFX extenderMotor;
  private final IntakeExtensionIO io;
  private final IntakeExtensionIOInputsAutoLogged replayedInputs =
      new IntakeExtensionIOInputsAutoLogged();
  private boolean extenderAtPosition = true;

  public IntakeExtension(IntakeExtensionIO io) {
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    extenderMotor.getConfigurator().apply(IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG);
    extenderMotor.setPosition(Rotation.of(0)); // intake should be fully retracted on bootup

    this.io = io;
    this.io.setMotor(extenderMotor);
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

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

  public void extend() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT));
  }

  public void retract() {
    extenderAtPosition = false;
    io.setExtensionSetpoint(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.IN));
  }

  public void setExtenderVoltage(Voltage extensionVoltage) {
    io.setExtenderVoltage(extensionVoltage);
  }

  public void stopMotor() {
    io.setExtenderVoltage(Volts.of(0));
  }

  public Angle getSetpoint() {
    return replayedInputs.extenderMotorSetpoint;
  }
}
