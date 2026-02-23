package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtensionIO {

  @AutoLog
  class IntakeExtensionIOInputs {
    public Voltage extenderMotorVoltage = Volts.of(0);
    public AngularVelocity extenderMotorRPS = RotationsPerSecond.of(0);
    public Current extenderMotorCurrent = Amps.of(0);
    public Angle extenderMotorPosition = Rotation.of(0);
    public Angle extenderMotorSetpoint = Rotation.of(0);
  }

  /** Sets the TalonFX motor configuraiton in the IO implementation. */
  default void setMotor(TalonFX motor) {}

  default void updateState(IntakeExtensionIOInputs inputs) {}

  default void setExtenderVoltage(Voltage extensionVoltage) {}

  default void setExtensionSetpoint(Angle setpoint) {}
}
