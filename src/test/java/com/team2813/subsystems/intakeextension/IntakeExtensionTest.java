package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
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
  @Order(1)
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
  @Order(2)
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
  @Order(3)
  public void testIntakeExtensionIsAtPositionExtend() {
    intakeExtension.extend();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertTrue("Extender should be at position after extending", intakeExtension.isExtenderAtPosition());
  }

  @Test
  @Order(4)
  public void testIntakeExtensionIsAtPositionRetract() {
    intakeExtension.retract();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertTrue(intakeExtension.isExtenderAtPosition());
  }
}
