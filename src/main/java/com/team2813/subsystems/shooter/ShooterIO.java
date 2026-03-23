package com.team2813.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double rightMainShooterMotorVoltageVolts = 0;
    public double rightMainShooterMotorAngleRotations = 0;
    public double rightMainShooterMotorRotPerSec = 0;
    public double rightMainShooterMotorStatorCurrentAmps = 0;
    public double rightMainShooterMotorSupplyCurrentAmps = 0;
    public double rightMainShooterSetpointRotsPerSec = 0;

    public double leftFollowerShooterMotorVoltageVolts = 0;
    public double leftFollowerShooterMotorRotPerSec = 0;
    public double leftFollowerShooterMotorStatorCurrentAmps = 0;
    public double leftFollowerShooterMotorSupplyCurrentAmps = 0;
  }

  /**
   * Updates Advantage kit autologged input data, as well as any other necessary states (like in
   * sim)
   *
   * @param inputs The "struct" (data class) to handle hardware inputs.
   */
  default void updateState(ShooterIOInputs inputs) {}

  default void setShooterMotorVelocity(AngularVelocity shooterMotorVelocity) {}

  default void setShooterMotorVoltage(Voltage shooterMotorVoltage) {}
}
