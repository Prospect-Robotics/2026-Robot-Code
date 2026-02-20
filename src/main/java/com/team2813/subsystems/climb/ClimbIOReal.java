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

    inputs.leftCarriagePositionInches = getInnerCarriagePosition().in(Inches);
    inputs.leftMotorRotations = leftMotor.getPosition().getValueAsDouble();
    inputs.leftMotorVelocityRotsPerSecond = leftMotor.getVelocity().getValueAsDouble();
    inputs.leftMotorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();

    inputs.rightCarriagePositionInches = getOuterCarriagePosition().in(Inches);
    inputs.rightMotorRotations = rightMotor.getPosition().getValueAsDouble();
    inputs.rightMotorVelocityRotsPerSecond = rightMotor.getVelocity().getValueAsDouble();
    inputs.rightMotorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightMotorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setLeftMotorSetpoint(Angle setpoint) {
    leftMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setRightMotorSetpoint(Angle setpoint) {
    rightMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setLeftMotorVoltage(Voltage voltage) {
    leftMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public void setRightMotorVoltage(Voltage voltage) {
    rightMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public Angle getLeftMotorPosition() {
    return leftMotor.getPosition().getValue();
  }

  @Override
  public Angle getRightMotorPosition() {
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
