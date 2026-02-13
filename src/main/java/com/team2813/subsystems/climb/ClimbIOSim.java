package com.team2813.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Voltage;


public class ClimbIOSim implements ClimbIO {
    private final TalonFX ClimbMotor1;
  private final TalonFXSimState ClimbMotor1SimState;

  private final TalonFX ClimbMotor2;
  private final TalonFXSimState ClimbMotor2SimState;

  public ClimbIOSim() {
    ClimbMotor1 = new TalonFX(0);
    ClimbMotor2 = new TalonFX(0);
    ClimbMotor1SimState = ClimbMotor1.getSimState();
    ClimbMotor2SimState = ClimbMotor2.getSimState();

    ClimbMotor1.getConfigurator().apply(ClimbConstants.CLIMB_MOTOR_1_CONFIG);
    ClimbMotor2.getConfigurator().apply(ClimbConstants.CLIMB_MOTOR_2_CONFIG);

    
  }

  @Override
  public void updateState(ClimbIOInputs inputs) {
   
  }

  public void updateSimulation() {
    
  }

  @Override
  public void setMotorVoltage(Voltage intakeVoltage, Voltage extenderVoltage) {
    
  }

}
