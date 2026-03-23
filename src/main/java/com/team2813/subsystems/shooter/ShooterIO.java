package com.team2813.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double upperRightShooterMotorVoltageVolts = 0;
    public double upperRightShooterMotorAngleRotations = 0;
    public double upperRightShooterMotorRotPerSec = 0;
    public double upperRightShooterMotorStatorCurrentAmps = 0;
    public double upperRightShooterMotorSupplyCurrentAmps = 0;
    public double upperRightShooterSetpointRotsPerSec = 0;

    public double upperLeftShooterMotorVoltageVolts = 0;
    public double upperLeftShooterMotorRotPerSec = 0;
    public double upperLeftShooterMotorStatorCurrentAmps = 0;
    public double upperLeftShooterMotorSupplyCurrentAmps = 0;
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
