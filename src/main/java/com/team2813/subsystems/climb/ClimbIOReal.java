package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ClimbIOReal implements ClimbIO {

  // TODO(vdikov): Looks like IOReal and IOSim could share the motor instance.
  private TalonFX motor;

  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOReal() {}

  @Override
  public void setMotor(TalonFX motor) {
    this.motor = motor;
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
    // TODO: This conversion code might be a major thorn later, or maybe its fine, and the sim code
    // needs rework.
    inputs.leftCarriagePositionInches = getCarriagePosition().in(Inches);
    inputs.leftMotorRotations = motor.getPosition().getValueAsDouble();
    inputs.leftMotorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();
    inputs.leftMotorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.leftMotorVoltage = motor.getMotorVoltage().getValueAsDouble();

    inputs.rightCarriagePositionInches = getCarriagePosition().in(Inches);
    inputs.rightMotorRotations = motor.getPosition().getValueAsDouble();
    inputs.rightMotorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();
    inputs.rightMotorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.rightMotorVoltage = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setMotorSetpoint(Angle setpoint) {
    motor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    motor.setVoltage(voltage.magnitude());
  }

  @Override
  public Angle getMotorPosition() {
    return motor.getPosition().getValue();
  }

  @Override
  public Distance getCarriagePosition() {
    return motorRotationToCarriagePosition(motor.getPosition().getValue());
  }

  private static Distance motorRotationToCarriagePosition(Angle motorPosition) {
    return Inches.of(
            motorPosition.in(Rotations) * ClimbConstants.LEFTCLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION)
        .times(2);
  }
}
