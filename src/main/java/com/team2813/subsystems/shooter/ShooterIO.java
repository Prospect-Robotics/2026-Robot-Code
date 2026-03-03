package com.team2813.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double mainShooterMotorVoltageVolts = 0;
    public double mainShooterMotorAngleRotations = 0;
    public double mainShooterMotorRotPerSec = 0;
    public double mainShooterMotorCurrentAmps = 0;
    public double mainShooterSetpointRotsPerSec = 0;

    public double followerShooterMotorVoltageVolts = 0;
    public double followerShooterMotorRotPerSec = 0;
    public double followerShooterMotorCurrentAmps = 0;
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
