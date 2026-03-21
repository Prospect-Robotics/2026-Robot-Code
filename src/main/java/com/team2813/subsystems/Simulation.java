package com.team2813.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.LoggedRobot;

/** Utilities for working in simulation. */
public class Simulation {
  /**
   * The time period to use in simulation.
   *
   * <p>Currently set to the same time period as we use in the actual robot.
   */
  public static final double TIME_PERIOD = LoggedRobot.defaultPeriodSecs;

  /** Gets the voltage to apply to simulated motors. */
  public static Voltage getMotorSupplyVoltage() {
    return Volts.of(RobotController.getBatteryVoltage());
  }

  private Simulation() {
    throw new AssertionError("Not instantiable");
  }
}
