package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.*;

public class IntakeExtensionTest {

  private IntakeExtension intakeExtension;

  @BeforeEach
  public void setUp() {
    // Initialize HAL fresh for each test to avoid static state leakage
    HAL.initialize(500, 1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.setHALRuntimeType(RuntimeType.kSimulation.value);

    intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());
  }

  @AfterEach
  public void tearDown() {
    if (intakeExtension != null) {
      intakeExtension.close();
      intakeExtension = null;
    }
    // Shutdown HAL to clear all static device state
    HAL.shutdown();
  }

  @Test
  public void testIntakeExtensionExtend() {
    intakeExtension.extend();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertEquals(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT)
            .in(Rotations),
        intakeExtension.getSetpoint().in(Rotations),
        0.01);
  }

  @Test
  public void testIntakeExtensionRetract() {
    intakeExtension.retract();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertEquals(
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.IN)
            .in(Rotations),
        intakeExtension.getSetpoint().in(Rotations),
        0.01);
  }

  @Test
  public void testIntakeExtensionIsAtPositionExtend() {
    intakeExtension.extend();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertTrue(
        intakeExtension.isExtenderAtPosition(),
            "Extender should be at position after extending");
  }

  @Test
  public void testIntakeExtensionIsAtPositionRetract() {
    intakeExtension.retract();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertTrue(intakeExtension.isExtenderAtPosition());
  }
}
