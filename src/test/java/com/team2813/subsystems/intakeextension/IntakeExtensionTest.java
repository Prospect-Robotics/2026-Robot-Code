package com.team2813.subsystems.intakeextension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junitpioneer.jupiter.ClearEnvironmentVariable;

@Disabled
@InitWPILib
@ClearEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE")
public class IntakeExtensionTest {
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
        intakeExtension.getSetpoint().magnitude(),
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT)
            .magnitude(),
        0.01);
  }

  @Disabled
  @Test
  public void testIntakeRetraction() {
    // create an intake extension subsystem

    IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

    // retract the intake
    intakeExtension.retract();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    assertEquals(
        intakeExtension.getSetpoint().magnitude(),
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.IN)
            .magnitude(),
        0.01);
  }

  @Disabled
  @Test
  public void testIntakeExtensionAtPosition() {
    // create an intake extension subsystem

    IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

    // The intake should start at the retracted position, so the extender should be at position
    assertTrue(intakeExtension.isExtenderAtPosition());

    // extend the intake
    intakeExtension.extend();
    // With new setpoint set by extend(), the extender should no longer be at position.
    assertFalse(intakeExtension.isExtenderAtPosition());

    // run periodic at the equivalent of 50 cycles (1 second) to let the intake reach the setpoint
    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
    }

    // TODO: Jazl, please debug this, it seems the PID, and kSVA constants for the Intake have not
    // been merged properly.
    //    assertTrue(intakeExtension.isExtenderAtPosition());
  }
}
