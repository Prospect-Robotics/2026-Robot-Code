package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import org.junit.jupiter.api.*;

@InitWPILib
public class IntakeExtensionTest {

  @AutoClose
  private final IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

  @Test
  public void testIntakeExtensionExtend() {
    intakeExtension.extend();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertEquals(
        IntakeExtensionConstants.ExtenderPositions.OUT.getAngle().in(Rotations),
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
        IntakeExtensionConstants.ExtenderPositions.IN.getAngle().in(Rotations),
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
