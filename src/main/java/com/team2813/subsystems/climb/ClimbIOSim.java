package com.team2813.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Voltage;

public class ClimbIOSim implements ClimbIO {
  private final TalonFX leftClimbMotor;
  private final TalonFXSimState leftClimbMotorSimState;

  private final TalonFX rightClimbMotor;
  private final TalonFXSimState rightClimbMotorSimState;

  public ClimbIOSim() {
    leftClimbMotor = new TalonFX(0);
    rightClimbMotor = new TalonFX(0);
    leftClimbMotorSimState = leftClimbMotor.getSimState();
    rightClimbMotorSimState = rightClimbMotor.getSimState();

    leftClimbMotor.getConfigurator().apply(ClimbConstants.LEFT_CLIMB_MOTOR_CONFIG);
    rightClimbMotor.getConfigurator().apply(ClimbConstants.RIGHT_CLIMB_MOTOR_CONFIG);
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {}

  public void updateSimulation() {}

  @Override
  public void setMotorVoltage(Voltage intakeVoltage, Voltage extenderVoltage) {}
}
