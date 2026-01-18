package com.team2813.subsystems.hopper;

import static com.team2813.subsystems.hopper.HopperConstants.HOTDOG_ROLLER_MOTOR_CONFIG;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team2813.Constants;
import com.team2813.subsystems.shooter.IntakeConstants;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
  private TalonFX rollerMotor;
  private TalonFX mainFeederMotor; // Left motor when seen from the front (intake side).
  private TalonFX followerFeederMotor; // Right motor when seen from the front (intake side).

  public HopperIOReal() {
    rollerMotor = new TalonFX(Constants.ROLLER_MOTOR_CAN_ID);
    rollerMotor.getConfigurator().apply(HOTDOG_ROLLER_MOTOR_CONFIG);

    mainFeederMotor = new TalonFX(Constants.MAIN_INTAKE_MOTOR_ID);
    mainFeederMotor.getConfigurator().apply(IntakeConstants.MAIN_INTAKE_MOTOR_CONFIG);

    followerFeederMotor = new TalonFX(Constants.FOLLOWER_INTAKE_MOTOR_ID);
    followerFeederMotor.setControl(
      new Follower(Constants.MAIN_INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));

  }

  @Override
  public void updateState(HopperIOInputs inputs) {
    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValue();
    inputs.rollerMotorVoltage = rollerMotor.getMotorVoltage().getValue();
    inputs.rollerMotorRPS = rollerMotor.getRotorVelocity().getValue();
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    rollerMotor.setVoltage(voltage.in(Volts));
    mainFeederMotor.setVoltage(voltage.in(Volts));
  }
}
