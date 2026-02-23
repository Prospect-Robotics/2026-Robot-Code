package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public Voltage mainShooterMotorVoltage = Volts.of(0);
    public Angle mainShooterMotorAngle = Rotations.of(0);
    public AngularVelocity mainShooterMotorRotPerSec = RotationsPerSecond.of(0);
    public Current mainShooterMotorCurrent = Amps.of(0);

    public Voltage followerShooterMotorVoltage = Volts.of(0);
    public AngularVelocity followerShooterMotorRotPerSec = RotationsPerSecond.of(0);
    public Current followerShooterMotorCurrent = Amps.of(0);

    public Voltage kickerMotorVoltage = Volts.of(0);
    public AngularVelocity kickerMotorRotPerSec = RotationsPerSecond.of(0);
    public Current kickerMotorCurrent = Amps.of(0);
  }

  /**
   * Updates Advantage kit autologged input data, as well as any other necessary states (like in
   * sim)
   *
   * @param inputs The "struct" (data class) to handle hardware inputs.
   */
  default void updateState(ShooterIOInputs inputs) {}

  default void setMotorVoltages(Voltage shooterVoltage, Voltage kickerVoltage) {
    setShooterMotorVoltage(shooterVoltage);
    setKickerMotorVoltage(kickerVoltage);
  }

  default void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {}

  default void setShooterMotorVoltage(Voltage shooterMotorVoltage) {}

  default void setKickerMotorVoltage(Voltage kickerMotorVoltage) {}
}
