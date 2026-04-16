package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class HoodIOSim implements HoodIO {
  private final TalonFX hoodMotor;
  private final TalonFXSimState hoodMotorSimState;

  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final SingleJointedArmSim hoodSim;

  private Angle motorSetpoint = Rotations.of(0);

  public HoodIOSim() {
    hoodMotor = new TalonFX(Constants.HOOD_MOTOR_ID);
    hoodMotor.getConfigurator().apply(HoodConstants.PIVOT_MOTOR_CONFIG);

    hoodMotorSimState = hoodMotor.getSimState();

    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            HoodConstants.HOOD_GEAR_RATIO,
            HoodConstants.HOOD_MOI.in(KilogramSquareMeters),
            HoodConstants.SIM_ARM_LENGTH.in(Meters),
            HoodConstants.MINIMUM_SHOOTER_ANGLE.in(Radians),
            HoodConstants.MAXIMUM_SHOOTER_ANGLE.in(Radians),
            true,
            HoodConstants.MINIMUM_SHOOTER_ANGLE.in(Radians));
  }

  @Override
  public void updateState(HoodIOInputs inputs) {
    hoodSim.setInputVoltage(inputs.motorVoltage.in(Volts));
    Logger.recordOutput("Hood/HoodAngleDegrees", Radians.of(hoodSim.getAngleRads()).in(Degree));

    hoodSim.update(Constants.SIM_TIME_PERIOD);

    hoodMotorSimState.setSupplyVoltage(Volts.of(12));

    hoodMotor.setVoltage(inputs.motorVoltage.in(Volts));
    hoodMotorSimState.setRawRotorPosition(
        Radians.of(hoodSim.getAngleRads()).times(HoodConstants.HOOD_GEAR_RATIO));
    hoodMotorSimState.setRotorVelocity(
        RadiansPerSecond.of(hoodSim.getVelocityRadPerSec()).times(HoodConstants.HOOD_GEAR_RATIO));

    SimulationVisualizer.getInstance().updateShooterHoodAngle(Radians.of(hoodSim.getAngleRads()));

    inputs.motorVoltage = hoodMotor.getMotorVoltage().getValue();
    inputs.motorVelocity = hoodMotor.getVelocity().getValue();
    inputs.motorStatorCurrent = hoodMotor.getStatorCurrent().getValue();
    inputs.motorSupplyCurrent = hoodMotor.getSupplyCurrent().getValue();
    inputs.motorSetpoint = motorSetpoint;
    inputs.motorAngle = hoodMotor.getPosition().getValue();
  }

  @Override
  public void setSetpoint(Angle angle) {
    Logger.recordOutput("Hood/SimMotorSetpointDegrees", angle.in(Degree));
    motorSetpoint = angle;
    hoodMotor.setControl(positionVoltage.withPosition(angle));
  }

  @Override
  public void stop() {
    hoodMotor.disable();
  }

  @Override
  public void close() {
    hoodMotor.close();
  }
}
