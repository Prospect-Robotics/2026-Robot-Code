package com.team2813.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
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

  /**
   * Gets the simulated state for the given motor, respecting the orientation.
   *
   * @param motor the motor to use to get the sim state; it should be configured
   * @param motorType the type of the motor to simulate.
   * @return the sim state of the motor
   */
  public static TalonFXSimState getConfiguredSimState(
      TalonFX motor, TalonFXSimState.MotorType motorType) {
    TalonFXSimState simState = motor.getSimState();
    simState.Orientation = getOrientation(motor);
    simState.setMotorType(motorType);
    return simState;
  }

  private static ChassisReference getOrientation(TalonFX motor) {
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    // Populate "outputConfigs" with the current configuration of the motor.
    motor.getConfigurator().refresh(outputConfigs);

    return switch (outputConfigs.Inverted) {
      case Clockwise_Positive -> ChassisReference.Clockwise_Positive;
      case CounterClockwise_Positive -> ChassisReference.CounterClockwise_Positive;
    };
  }

  private Simulation() {
    throw new AssertionError("Not instantiable");
  }
}
