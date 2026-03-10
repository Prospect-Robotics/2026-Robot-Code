package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ClimbIOReal extends ClimbIO {

  private TalonFX climbMotor;

  private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

  public ClimbIOReal(AllClimbConstants climbConstants) {
    super(climbConstants);

    climbMotor = new TalonFX(super.climbConstants.climbCanID());
    climbMotor.getConfigurator().apply(super.climbConstants.climbMotorConfig());
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
    inputs.carriagePositionInches = getCarriagePosition().in(Inches);
    inputs.motorRotations = climbMotor.getPosition().getValueAsDouble();
    inputs.motorVelocityRotsPerSecond = climbMotor.getVelocity().getValue();
    inputs.motorCurrent = climbMotor.getStatorCurrent().getValueAsDouble();
    inputs.motorVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setMotorSetpoint(Angle setpoint) {
    climbMotor.setControl(positionControl.withPosition(setpoint));
  }

  @Override
  public void stopMotor() {
    climbMotor.disable();
  }

  @Override
  public Angle getMotorPosition() {
    return climbMotor.getPosition().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage motorVoltage) {
    climbMotor.setVoltage(motorVoltage.in(Volts));
  }

  @Override
  public Distance getCarriagePosition() {
    return motorRotationToCarriagePosition(climbMotor.getPosition().getValue());
  }

  private Distance motorRotationToCarriagePosition(Angle motorPosition) {
    return super.climbConstants.climbHeightChangePerRotation().times(
        motorPosition.in(Rotations));
  }
}
