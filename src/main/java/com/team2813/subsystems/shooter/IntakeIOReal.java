package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2813.Constants;
import com.team2813.subsystems.hopper.HopperIO.HopperIOInputs;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
  private TalonFX mainIntakeMotor; // Left motor when seen from the front (intake side).
  private TalonFX followerIntakeMotor; // Right motor when seen from the front (intake side).

  public IntakeIOReal() {
    mainIntakeMotor = new TalonFX(Constants.MAIN_INTAKE_MOTOR_ID);
    mainIntakeMotor.getConfigurator().apply(IntakeConstants.MAIN_INTAKE_MOTOR_CONFIG);

    followerIntakeMotor = new TalonFX(Constants.FOLLOWER_INTAKE_MOTOR_ID);
    followerIntakeMotor.setControl(
        new Follower(Constants.MAIN_INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateState(HopperIOInputs inputs) {}

  @Override
  public void setMotorVoltage(Voltage voltage) {
    mainIntakeMotor.setVoltage(voltage.in(Volts));
  }
}
