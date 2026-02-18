package com.team2813.subsystems.intakeextension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team2813.Constants;
import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

@InitWPILib
public class IntakeExtensionTest {
  @BeforeAll
  public static void verifyNotInReplayMode() {
    assertTrue(
        Constants.currentMode != Constants.Mode.REPLAY, "Must not be in replay mode to run tests");
  }
  
  @Test
  public void testIntakeExtension() {
    // create an intake extension subsystem

    IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

    // extend the intake
    intakeExtension.extend();

    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
      SimHooks.stepTiming(0.02);
    }

    assertEquals(
        intakeExtension.getSetpoint().magnitude(),
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.OUT)
            .magnitude(),
        0.01);
  }

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
      SimHooks.stepTiming(0.02);
    }

    assertEquals(
        intakeExtension.getSetpoint().magnitude(),
        IntakeExtensionConstants.toMotorSetpoint(IntakeExtensionConstants.ExtenderPositions.IN)
            .magnitude(),
        0.01);
  }

  @Test
  public void testIntakeExtensionAtPosition() {
    Assumptions.assumeTrue(
        Constants.simMode == Constants.Mode.SIM, "Must be in sim mode to run tests");
    // create an intake extension subsystem

    IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

    // extend the intake
    intakeExtension.extend();

    // run periodic at the equivalent of 50 cycles (1 second) to let the intake reach the setpoint
    for (int i = 0; i < 50; i++) {
      intakeExtension.periodic();
      SimHooks.stepTiming(0.02);
    }

    assertTrue(intakeExtension.isExtenderAtPosition());
  }
}
