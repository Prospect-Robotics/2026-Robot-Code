package com.team2813.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team2813.util.PhoenixUtil;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Drive Gyroscope simulation.
 *
 * <p>Implementation borrowed from
 * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/subsystems/drive/GyroIOSim.java
 */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
