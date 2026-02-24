package com.team2813.subsystems.intakeextension;


import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeAll;

public class IntakeExtensionTest {

  @BeforeAll
  public static void init() {
    HAL.initialize(500, 1);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.setHALRuntimeType(RuntimeType.kSimulation.value);
  }


}