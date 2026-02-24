package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private static final double SUPPLY_VOLTAGE = 12.0;

  private TalonFX intakeMotor;
  private TalonFXSimState intakeSimState;

  private FlywheelSim intakeFlywheelSim;

  // Track the actual motor output voltage (clamped to supply voltage limits)
  private double actualMotorVoltage = 0.0;

  public IntakeRollerIOSim() {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);
    intakeMotor.getConfigurator().apply(IntakeRollerConstants.INTAKE_MOTOR_CONFIG);

    intakeSimState = intakeMotor.getSimState();

    intakeFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                IntakeRollerConstants.INTAKE_SIM_MOI,
                IntakeRollerConstants.INTAKE_MOTOR_TO_INTAKE_GEARING),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateState(IntakeRollerIOInputs inputs) {
    updateSimulation();

    intakeSimState.setSupplyVoltage(SUPPLY_VOLTAGE);

    // Report the actual motor output voltage
    inputs.intakeMotorVoltage = Volts.of(actualMotorVoltage);
    inputs.intakeMotorRPS = intakeMotor.getVelocity().getValue();
    inputs.intakeMotorCurrent = intakeMotor.getStatorCurrent().getValue();
  }

  public void updateSimulation() {
    intakeFlywheelSim.update(Constants.SIM_TIME_PERIOD);

    intakeSimState.setRotorAcceleration(intakeFlywheelSim.getAngularAcceleration());
    intakeSimState.setRotorVelocity(intakeFlywheelSim.getAngularVelocity());
  }

  @Override
  public void setIntakeMotorVoltage(Voltage intakeMotorVoltage) {
    // Clamp to supply voltage to get actual motor output
    actualMotorVoltage = MathUtil.clamp(intakeMotorVoltage.in(Volts), -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);

    intakeMotor.setVoltage(actualMotorVoltage);
    intakeFlywheelSim.setInputVoltage(actualMotorVoltage);
  }
}
