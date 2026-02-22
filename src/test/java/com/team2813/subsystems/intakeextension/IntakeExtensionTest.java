package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team2813.Constants;
import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
// import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

// @Disabled
@InitWPILib
public class IntakeExtensionTest {
  @BeforeAll
  public static void verifyNotInReplayMode() {
    assertTrue(
        Constants.currentMode != Constants.Mode.REPLAY, "Must not be in replay mode to run tests");

    assertTrue(HAL.initialize(500, 0)); // initialize the HAL, crash if failed
  }

  @Disabled
  @Test
  public void testIntakeExtension() {
    // create an intake extension subsystem

    IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

    // extend the intake
    intakeExtension.extend();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertEquals(
        intakeExtension.getPosition().in(Rotations),
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT)
            .in(Rotations),
        0.4);

    intakeExtension.close();
  }

  @Disabled
  @Test
  public void testIntakeRetraction() {
    Assumptions.assumeTrue(
        Constants.simMode == Constants.Mode.SIM, "Must be in sim mode to run tests");
    // create an intake extension subsystem

    IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

    // retract the intake
    intakeExtension.retract();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    System.out.println(
        "Intake Motor Position: " + intakeExtension.getPosition().in(Rotations) + " rotations");
    System.out.println(
        "Intake Motor Setpoint: " + intakeExtension.getSetpoint().in(Rotations) + " rotations");

    assertEquals(
        intakeExtension.getPosition().in(Rotations),
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.IN)
            .in(Rotations),
        0.4);

    intakeExtension.close();
  }

  @Disabled
  @Test
  public void testIntakeExtensionAtPosition() {
    Assumptions.assumeTrue(
        Constants.simMode == Constants.Mode.SIM, "Must be in sim mode to run tests");
    // create an intake extension subsystem

    IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

    // The intake should start at the retracted position, so the extender should be at position
    assertTrue(intakeExtension.isExtenderAtPosition());

    // extend the intake
    intakeExtension.extend();
    // With new setpoint set by extend(), the extender should no longer be at position.
    assertFalse(intakeExtension.isExtenderAtPosition());

    System.out.println(
        "Intake Motor Position (rot): " + intakeExtension.getPosition().in(Rotations));
    System.out.println(
        "Intake Motor Setpoint (rot): " + intakeExtension.getSetpoint().in(Rotations));

    // run periodic at the equivalent of 50 cycles (1 second) to let the intake reach the setpoint
    for (int i = 0; i < 50; i++) {
      System.out.print(
          "Intake Motor Position (rot): "
              + String.format("%5.2f", intakeExtension.getPosition().in(Rotations))
              + ", ");

      intakeExtension.periodic();
    }

    System.out.println(
        "Intake Motor Position (rot): " + intakeExtension.getPosition().in(Rotations));
    System.out.println(
        "Intake Motor Setpoint (rot): " + intakeExtension.getSetpoint().in(Rotations));

    assertTrue(intakeExtension.isExtenderAtPosition());

    intakeExtension.close();
  }
}
