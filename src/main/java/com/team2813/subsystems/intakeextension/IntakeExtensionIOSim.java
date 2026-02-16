package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class IntakeExtensionIOSim implements IntakeExtensionIO {

  private final TalonFX extenderMotor;
  private final TalonFXSimState extenderMotorSimState;

  private final ElevatorSim extenderSim;

  private Angle extensionSetpoint;

  public IntakeExtensionIOSim() {
    extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_CAN_ID);
    extenderMotorSimState = extenderMotor.getSimState();

    extenderMotor.getConfigurator().apply(IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG);

    extenderSim =
        new ElevatorSim(
            DCMotor.getKrakenX44(1),
            IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING,
            IntakeExtensionConstants.WEIGHT_OF_EXTENDER_CARRIAGE.in(Kilograms),
            IntakeExtensionConstants.PULLEY_RADIUS.in(Meters),
            IntakeExtensionConstants.UNEXTENDED_POSITION.in(Meters),
            IntakeExtensionConstants.EXTENDED_POSITION.in(Meters),
            true,
            0); // Start unextended

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
            / (2.0 * Math.PI * IntakeExtensionConstants.PULLEY_RADIUS.in(Meters))
            * IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING);
    extenderMotorSimState.setRawRotorPosition(
        Units.radiansToRotations(
            extenderSim.getOutput(0)
                / (2.0 * Math.PI * IntakeExtensionConstants.PULLEY_RADIUS.in(Meters))
                * IntakeExtensionConstants.EXTENDER_MOTOR_TO_EXTENDER_GEARING));
  }
}
