package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.*;

@InitWPILib
public class IntakeExtensionTest {

  private IntakeExtension intakeExtension;

  @BeforeEach
  public void setUp() {
    intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());
  }

  @AfterEach
  public void tearDown() {
    if (intakeExtension != null) {
      try {
        intakeExtension.close();
      } catch (Exception e) {
        DriverStation.reportError(
            "Failed to close IntakeExtension IO: " + e.getMessage(), e.getStackTrace());
      }
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
        intakeExtension.getPosition().in(Rotations),
        0.5);
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
        intakeExtension.getPosition().in(Rotations),
        0.5);
  }

  @Test
  public void testIntakeExtensionIsAtPositionExtend() {
    intakeExtension.extend();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertTrue(
        intakeExtension.isExtenderAtPosition(), "Extender should be at position after extending");
  }

  @Test
  public void testIntakeExtensionIsAtPositionRetract() {
    intakeExtension.retract();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertTrue(
        intakeExtension.isExtenderAtPosition(), "Extender should be at position after retracting");
  }
}
