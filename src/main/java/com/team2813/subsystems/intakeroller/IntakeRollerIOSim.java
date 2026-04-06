package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private TalonFX leftIntakeMotor;
  private TalonFXSimState leftIntakeSimState;

  private TalonFX rightIntakeMotor;
  private TalonFXSimState rightIntakeSimState;

  private FlywheelSim intakeFlywheelSim;

  public IntakeRollerIOSim() {
    leftIntakeMotor = new TalonFX(Constants.LEFT_INTAKE_MOTOR_CAN_ID);
    leftIntakeMotor.getConfigurator().apply(IntakeRollerConstants.LEFT_INTAKE_MOTOR_CONFIG);

    rightIntakeMotor = new TalonFX(Constants.RIGHT_INTAKE_MOTOR_CAN_ID);
    rightIntakeMotor.getConfigurator().apply(IntakeRollerConstants.RIGHT_INTAKE_MOTOR_CONFIG);

    leftIntakeSimState = leftIntakeMotor.getSimState();
    rightIntakeSimState = rightIntakeMotor.getSimState();

    intakeFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                IntakeRollerConstants.INTAKE_SIM_MOI,
                IntakeRollerConstants.INTAKE_MOTOR_TO_INTAKE_GEARING),
            DCMotor.getKrakenX60(2));
  }

  @Override
  public void updateState(IntakeRollerIOInputs inputs) {
    updateSimulation();

    leftIntakeSimState.setSupplyVoltage(Volts.of(12));
    rightIntakeSimState.setSupplyVoltage(Volts.of(12));

    inputs.leftIntakeMotorVoltage = leftIntakeMotor.getMotorVoltage().getValue();
    inputs.leftIntakeMotorRPS = leftIntakeMotor.getVelocity().getValue();
    inputs.leftIntakeMotorStatorCurrent = leftIntakeMotor.getStatorCurrent().getValue();
    inputs.leftIntakeMotorSupplyCurrent = leftIntakeMotor.getSupplyCurrent().getValue();

    inputs.rightIntakeMotorVoltage = rightIntakeMotor.getMotorVoltage().getValue();
    inputs.rightIntakeMotorRPS = rightIntakeMotor.getVelocity().getValue();
    inputs.rightIntakeMotorStatorCurrent = rightIntakeMotor.getStatorCurrent().getValue();
    inputs.rightIntakeMotorSupplyCurrent = rightIntakeMotor.getSupplyCurrent().getValue();
  }

  public void updateSimulation() {
    // TODO: Once we fetch from main, change this to the SIM_TIME in Constants.
    intakeFlywheelSim.update(0.02);

    leftIntakeSimState.setRotorAcceleration(intakeFlywheelSim.getAngularAcceleration());
    leftIntakeSimState.setRotorVelocity(intakeFlywheelSim.getAngularVelocity());

    rightIntakeSimState.setRotorAcceleration(intakeFlywheelSim.getAngularAcceleration());
    rightIntakeSimState.setRotorVelocity(intakeFlywheelSim.getAngularVelocity());
  }

  @Override
  public void setIntakeMotorVoltage(Voltage intakeMotorVoltage) {
    leftIntakeMotor.setVoltage(intakeMotorVoltage.in(Volts));
    rightIntakeMotor.setVoltage(intakeMotorVoltage.in(Volts));

    intakeFlywheelSim.setInputVoltage(intakeMotorVoltage.in(Volts));
  }
}
