package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2813.Constants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ClimbIOReal extends ClimbIO {

  private TalonFX innerMotor;
  private TalonFX outerMotor;

  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOReal(AllClimbConstants climbConstants) {
    super(climbConstants);
    innerMotor = new TalonFX(Constants.LEFTCLIMB_MOTOR_ID);
    innerMotor.getConfigurator().apply(ClimbConstants.INNER_MOTOR_TO_CLIMB_CONFIG);
    outerMotor = new TalonFX(Constants.RIGHTCLIMB_MOTOR_ID);
    outerMotor.getConfigurator().apply(ClimbConstants.OUTER_MOTOR_TO_CLIMB_CONFIG);
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {

    inputs.innerCarriagePositionInches = getInnerCarriagePosition().in(Inches);
    inputs.innerMotorRotations = innerMotor.getPosition().getValueAsDouble();
    inputs.innerMotorVelocityRotsPerSecond = innerMotor.getVelocity().getValue();
    inputs.innerMotorCurrent = innerMotor.getStatorCurrent().getValueAsDouble();
    inputs.innerMotorVoltage = innerMotor.getMotorVoltage().getValueAsDouble();

    inputs.outerCarriagePositionInches = getOuterCarriagePosition().in(Inches);
    inputs.outerMotorRotations = outerMotor.getPosition().getValueAsDouble();
    inputs.outerMotorVelocityRotsPerSecond = outerMotor.getVelocity().getValue();
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
  public void setInnerMotorVoltage(Voltage motorVoltage) {
    innerMotor.setVoltage(motorVoltage.in(Volts));
  }

  @Override
  public void setOuterMotorVoltage(Voltage motorVoltage) {
    outerMotor.setVoltage(motorVoltage.in(Volts));
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
    return ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.times(
        motorPosition.in(Rotations));
  }

  private static Distance rightMotorRotationToCarriagePosition(Angle motorPosition) {
    return ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.times(
        motorPosition.in(Rotations));
  }
}
