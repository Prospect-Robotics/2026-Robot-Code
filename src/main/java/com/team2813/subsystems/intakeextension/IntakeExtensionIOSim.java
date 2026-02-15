package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class IntakeExtensionIOSim implements IntakeExtensionIO {

  private final TalonFX extenderMotor;
  private final TalonFXSimState extenderMotorSimState;

  private final LinearSystemSim<N2, N1, N2> extenderSim;

  private Angle extensionSetpoint;

  public IntakeExtensionIOSim() {
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    extenderMotorSimState = extenderMotor.getSimState();

    extenderMotor.getConfigurator().apply(IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG);

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
    extenderMotorSimState.setSupplyVoltage(12);

    inputs.extenderMotorVoltage = extenderMotor.getMotorVoltage().getValue();
    inputs.extenderMotorRPS = extenderMotor.getRotorVelocity().getValue();
    inputs.extenderMotorCurrent = extenderMotor.getStatorCurrent().getValue();
    inputs.extenderMotorPosition = extenderMotor.getPosition().getValue();
    inputs.extenderMotorSetpoint = extensionSetpoint;
  }

  public void updateSimulation() {
    extenderSim.setInput(extenderMotorSimState.getMotorVoltage());

    extenderSim.update(0.02);

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
}
