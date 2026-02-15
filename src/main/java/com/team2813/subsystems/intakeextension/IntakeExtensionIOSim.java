package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class IntakeExtensionIOSim implements IntakeExtensionIO {
  private final TalonFX intakeMotor;
  private final TalonFXSimState intakeMotorSimState;

  private final TalonFX extenderMotor;
  private final TalonFXSimState extenderMotorSimState;

  private final FlywheelSim intakeSim;
  private final LinearSystemSim<N2, N1, N2> extenderSim;

  private Angle extensionSetpoint;

  public IntakeExtensionIOSim() {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    intakeMotorSimState = intakeMotor.getSimState();
    extenderMotorSimState = extenderMotor.getSimState();

    intakeMotor.getConfigurator().apply(IntakeExtensionConstants.INTAKE_MOTOR_CONFIG);
    extenderMotor.getConfigurator().apply(IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG);

    intakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                0.000157147494,
                IntakeExtensionConstants.INTAKE_MOTOR_TO_INTAKE_GEARING),
            DCMotor.getKrakenX60(1),
            0.0);

    extenderSim =
        new LinearSystemSim<N2, N1, N2>(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(1),
                5.0,
                IntakeExtensionConstants.PULLEY_RADIUS_METERS,
                IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING),
            0.0,
            0.0);

    extensionSetpoint = Rotation.of(0);
  }

  @Override
  public void updateState(IntakeExtensionIOInputs inputs) {
    updateSimulation();
    intakeMotorSimState.setSupplyVoltage(12);
    extenderMotorSimState.setSupplyVoltage(12);

    inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValue();
    inputs.intakeMotorRPS = intakeMotor.getRotorVelocity().getValue();
    inputs.intakeMotorCurrent = intakeMotor.getStatorCurrent().getValue();

    inputs.extenderMotorVoltage = extenderMotor.getMotorVoltage().getValue();
    inputs.extenderMotorRPS = extenderMotor.getRotorVelocity().getValue();
    inputs.extenderMotorCurrent = extenderMotor.getStatorCurrent().getValue();
    inputs.extenderMotorPosition = extenderMotor.getPosition().getValue();
    inputs.extenderMotorSetpoint = extensionSetpoint;
  }

  public void updateSimulation() {
    intakeSim.setInput(intakeMotorSimState.getMotorVoltage());
    extenderSim.setInput(extenderMotorSimState.getMotorVoltage());

    intakeSim.update(0.02);
    extenderSim.update(0.02);

    intakeMotorSimState.setRotorAcceleration(intakeSim.getAngularAcceleration());
    intakeMotorSimState.setRotorVelocity(intakeSim.getAngularVelocity());

    extenderMotorSimState.setRotorVelocity(
        extenderSim.getOutput(1)
            / (2.0 * Math.PI * IntakeExtensionConstants.PULLEY_RADIUS_METERS)
            * IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING);
    extenderMotorSimState.setRawRotorPosition(
        Units.radiansToRotations(
            extenderSim.getOutput(0)
                / (2.0 * Math.PI * IntakeExtensionConstants.PULLEY_RADIUS_METERS)
                * IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING));
  }

  @Override
  public void setIntakeVoltage(Voltage intakeVoltage) {
    intakeMotor.setVoltage(intakeVoltage.in(Volts));
  }
}
