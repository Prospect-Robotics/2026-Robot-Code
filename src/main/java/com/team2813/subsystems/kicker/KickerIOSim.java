package com.team2813.subsystems.kicker;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class KickerIOSim implements KickerIO {
  private final FlywheelSim upperMotorFlywheelSim;
  private final FlywheelSim lowerMotorFlywheelSim;
  private final TalonFX upperKickerMotor;
  private final TalonFX lowerKickerMotor;

  public KickerIOSim() {
    upperKickerMotor = new TalonFX(Constants.UPPER_KICKER_MOTOR_ID);
    upperKickerMotor.getConfigurator().apply(KickerConstants.UPPER_KICKER_MOTOR_CONFIG);
    lowerKickerMotor = new TalonFX(Constants.LOWER_KICKER_MOTOR_ID);
    lowerKickerMotor.getConfigurator().apply(KickerConstants.LOWER_KICKER_MOTOR_CONFIG);

    upperMotorFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                KickerConstants.KICKER_SIM_MOI.in(
                    KilogramSquareMeters), // "Moment of Inertia" taken from OnShape.
                KickerConstants.UPPER_MOTOR_GEARING),
            DCMotor.getKrakenX60(1));

    lowerMotorFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                KickerConstants.KICKER_SIM_MOI.in(
                    KilogramSquareMeters), // "Moment of Inertia" taken from OnShape.
                KickerConstants.LOWER_MOTOR_GEARING),
            DCMotor.getKrakenX60(1));
  }

  @Override
    public void setMotorVoltage(Voltage upperKickerMotorVoltage, Voltage lowerKickerMotorVoltage) {
        upperKickerMotor.setVoltage(upperKickerMotorVoltage.in(Volts));
        lowerKickerMotor.setVoltage(lowerKickerMotorVoltage.in(Volts));
    }

  @Override
  public void updateState(KickerIOInputs inputs) {
    updateSimulation();

    inputs.upperMotorVoltage = upperKickerMotor.getMotorVoltage().getValue();
    inputs.upperMotorRotationalVelocity = upperKickerMotor.getVelocity().getValue();
    inputs.upperMotorStatorCurrent = upperKickerMotor.getStatorCurrent().getValue();
    inputs.upperMotorSupplyCurrent = upperKickerMotor.getSupplyCurrent().getValue();

    inputs.lowerMotorVoltage = lowerKickerMotor.getMotorVoltage().getValue();
    inputs.lowerMotorRotationalVelocity = lowerKickerMotor.getVelocity().getValue();
    inputs.lowerMotorStatorCurrent = lowerKickerMotor.getStatorCurrent().getValue();
    inputs.lowerMotorSupplyCurrent = lowerKickerMotor.getSupplyCurrent().getValue();
  }

  private void updateSimulation() {
    upperMotorFlywheelSim.update(Constants.SIM_TIME_PERIOD);

    TalonFXSimState simState = motor.getSimState();
    simState.setRotorAcceleration(flywheelSim.getAngularAcceleration());
    simState.setRotorVelocity(flywheelSim.getAngularVelocity());
    simState.setSupplyVoltage(Volts.of(12));
  }

  @Override
  public void close() {
    motor.close();
  }
}
