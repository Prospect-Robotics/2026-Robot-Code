package com.team2813.subsystems.intakeroller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class IntakeRollerTest {

  private IntakeRoller intakeRoller;

  @BeforeEach
  public void setUp() {
    // Initialize HAL fresh for each test to avoid static state leakage
    HAL.initialize(500, 1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimHooks.setHALRuntimeType(RuntimeType.kSimulation.value);

    intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
  }

  @AfterEach
  public void tearDown() {
    intakeRoller = null;
    // Shutdown HAL to clear all static device state
    HAL.shutdown();
  }

  @Test
  @Order(1)
  public void intakeRollerIntakeTest() {
    intakeRoller.intake();

    for (int i = 0; i < 50; i++) {
      intakeRoller.periodic();
    }

    assertEquals(
        intakeRoller.getIntakeRollerVoltage(),
            8,
            0.01,
            "Intake roller voltage should be at least 5V for intake");
  }

  @Test
  @Order(2)
  public void intakeRollerOuttakeTest() {
    intakeRoller.outtake();

    for (int i = 0; i < 50; i++) {
      intakeRoller.periodic();
    }

    assertEquals(
        intakeRoller.getIntakeRollerVoltage(),
        -8,
        0.01,
        "Intake roller voltage should be at most -8V for outtake");
  }

  @Test
  @Order(3)
  public void intakeRollerStopTest() {
    intakeRoller.stop();

    for (int i = 0; i < 50; i++) {
      intakeRoller.periodic();
    }

    assertEquals(0, intakeRoller.getIntakeRollerVoltage(), 0.01);
  }
}
