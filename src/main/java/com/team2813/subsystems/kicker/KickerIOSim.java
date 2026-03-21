// todo re add sim code

package com.team2813.subsystems.kicker;

public class KickerIOSim implements KickerIO {
  /*
  private final FlywheelSim flywheelSim;
  private final TalonFX motor;

  public KickerIOSim() {
    motor = new TalonFX(Constants.KICKER_MOTOR_ID);
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                KickerConstants.KICKER_SIM_MOI.in(
                    KilogramSquareMeters), // "Moment of Inertia" taken from OnShape.
                KickerConstants.KICKER_MOTOR_TO_FLYWHEEL_GEARING),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void setMotorVoltage(Voltage kickerMotorVoltage) {
    double volts = kickerMotorVoltage.in(Volts);
    motor.setVoltage(volts);
    flywheelSim.setInputVoltage(volts);
  }

  @Override
  public void updateState(KickerIOInputs inputs) {
    updateSimulation();

    inputs.motorVoltage = motor.getMotorVoltage().getValue();
    inputs.motorRotationalVelocity = motor.getVelocity().getValue();
    inputs.motorCurrent = motor.getStatorCurrent().getValue();
  }

  private void updateSimulation() {
    flywheelSim.update(Constants.SIM_TIME_PERIOD);

    TalonFXSimState simState = motor.getSimState();
    simState.setRotorAcceleration(flywheelSim.getAngularAcceleration());
    simState.setRotorVelocity(flywheelSim.getAngularVelocity());
    simState.setSupplyVoltage(Volts.of(12));
  }

  @Override
  public void close() {
    motor.close();
  }
    */
}
