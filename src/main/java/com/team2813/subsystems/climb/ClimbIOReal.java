package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ClimbIOReal implements ClimbIO {

  private TalonFX innerMotor;
  private TalonFX outerMotor;

  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOReal() {
    innerMotor = new TalonFX(0);
    innerMotor.getConfigurator().apply(ClimbConstants.INNER_MOTOR_TO_CLIMB_CONFIG);
    outerMotor = new TalonFX(0);
    outerMotor.getConfigurator().apply(ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG);
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {

    inputs.innerCarriagePositionInches = getInnerCarriagePosition().in(Inches);
    inputs.innerMotorRotations = innerMotor.getPosition().getValueAsDouble();
    inputs.innerMotorVelocityRotsPerSecond = innerMotor.getVelocity().getValueAsDouble();
    inputs.innerMotorCurrent = innerMotor.getStatorCurrent().getValueAsDouble();
    inputs.innerMotorVoltage = innerMotor.getMotorVoltage().getValueAsDouble();

    inputs.outerCarriagePositionInches = getOuterCarriagePosition().in(Inches);
    inputs.outerMotorRotations = outerMotor.getPosition().getValueAsDouble();
    inputs.outerMotorVelocityRotsPerSecond = outerMotor.getVelocity().getValueAsDouble();
    inputs.outerMotorCurrent = outerMotor.getStatorCurrent().getValueAsDouble();
    inputs.outerMotorVoltage = outerMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setInnerMotorSetpoint(Angle setpoint) {
    innerMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void setOuterMotorSetpoint(Angle setpoint) {
    outerMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void stopInnerMotor() {
    innerMotor.disable();
  }

  @Override
  public void stopOuterMotor() {
    outerMotor.disable();
  }

  @Override
  public Angle getInnerMotorPosition() {
    return innerMotor.getPosition().getValue();
  }

  @Override
  public Angle getOuterMotorPosition() {
    return outerMotor.getPosition().getValue();
  }

  @Override
  public Distance getInnerCarriagePosition() {
    return leftMotorRotationToCarriagePosition(innerMotor.getPosition().getValue());
  }

  @Override
  public Distance getOuterCarriagePosition() {
    return rightMotorRotationToCarriagePosition(outerMotor.getPosition().getValue());
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
