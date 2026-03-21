package com.team2813.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double shooterMotor1VoltageVolts = 0;
    public double shooterMotor1AngleRotations = 0;
    public double shooterMotor1RotPerSec = 0;
    public double shooterMotor1CurrentAmps = 0;
    public double shooter1SetpointRotsPerSec = 0;

    public double shooterMotor2VoltageVolts = 0;
    public double shooterMotor2AngleRotations = 0;
    public double shooterMotor2RotPerSec = 0;
    public double shooterMotor2CurrentAmps = 0;
    public double shooter2SetpointRotsPerSec = 0;

    public double shooterMotor3VoltageVolts = 0;
    public double shooterMotor3AngleRotations = 0;
    public double shooterMotor3RotPerSec = 0;
    public double shooterMotor3CurrentAmps = 0;
    public double shooter3SetpointRotsPerSec = 0;

    public double shooterMotor4VoltageVolts = 0;
    public double shooterMotor4AngleRotations = 0;
    public double shooterMotor4RotPerSec = 0;
    public double shooterMotor4CurrentAmps = 0;
    public double shooter4SetpointRotsPerSec = 0;
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
