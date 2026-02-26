package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ClimbIOReal implements ClimbIO {

  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOReal() {
    leftMotor = new TalonFX(0);
    leftMotor.getConfigurator().apply(ClimbConstants.LEFT_MOTOR_TO_CLIMB_CONFIG);
    rightMotor = new TalonFX(0);
    rightMotor.getConfigurator().apply(ClimbConstants.RIGHT_MOTOR_TO_CLIMB_CONFIG);
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {

    inputs.innerCarriagePositionInches = getInnerCarriagePosition().in(Inches);
    inputs.innerMotorRotations = leftMotor.getPosition().getValueAsDouble();
    inputs.innerMotorVelocityRotsPerSecond = leftMotor.getVelocity().getValueAsDouble();
    inputs.innerMotorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.innerMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();

    inputs.outerCarriagePositionInches = getOuterCarriagePosition().in(Inches);
    inputs.outerMotorRotations = rightMotor.getPosition().getValueAsDouble();
    inputs.outerMotorVelocityRotsPerSecond = rightMotor.getVelocity().getValueAsDouble();
    inputs.outerMotorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.outerMotorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setInnerMotorSetpoint(Angle setpoint) {
    leftMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setOuterMotorSetpoint(Angle setpoint) {
    rightMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setInnerMotorVoltage(Voltage voltage) {
    leftMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public void setOuterMotorVoltage(Voltage voltage) {
    rightMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public Angle getInnerMotorPosition() {
    return leftMotor.getPosition().getValue();
  }

  @Override
  public Angle getOuterMotorPosition() {
    return rightMotor.getPosition().getValue();
  }

  @Override
  public Distance getInnerCarriagePosition() {
    return leftMotorRotationToCarriagePosition(leftMotor.getPosition().getValue());
  }

  @Override
  public Distance getOuterCarriagePosition() {
    return rightMotorRotationToCarriagePosition(rightMotor.getPosition().getValue());
  }

  private static Distance leftMotorRotationToCarriagePosition(Angle motorPosition) {
    return Inches.of(
            motorPosition.in(Rotations)
                * ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION)
        .times(2);
  }

  private static Distance rightMotorRotationToCarriagePosition(Angle motorPosition) {
    return Inches.of(
            motorPosition.in(Rotations)
                * ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION)
        .times(2);
  }
}
