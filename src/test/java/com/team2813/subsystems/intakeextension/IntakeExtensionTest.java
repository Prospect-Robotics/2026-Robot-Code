package com.team2813.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.*;

import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import com.team2813.subsystems.Simulation;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AutoClose;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/** Tests for {@link IntakeExtension}. */
@InitWPILib
public class IntakeExtensionTest {
  private static final double ACCEPTABLE_ERROR_IN_ROTATIONS =
      IntakeExtension.ACCEPTABLE_ERROR_IN_ROTATIONS * 1.1;

  @AutoClose
  private final IntakeExtension intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());

  @BeforeAll
  public static void initializeLogger() {
    Logger.disableConsoleCapture();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @Test
  public void testIntakeExtensionExtend() {
    intakeExtension.extend();

    simulateRunLoop(Units.Seconds.of(1));

    assertAll(
        () ->
            assertTrue(
                intakeExtension.isExtenderAtPosition(),
                "Extender should be at position after extending"),
        () ->
            assertEquals(
                IntakeExtensionConstants.ExtenderPositions.OUT.getAngle().in(Rotations),
                intakeExtension.getPosition().in(Rotations),
                ACCEPTABLE_ERROR_IN_ROTATIONS,
                "Extender should be close to the OUT position, in Rotations"));
  }

  @Test
  public void testIntakeExtensionRetract() {
    intakeExtension.retract();

    simulateRunLoop(Units.Seconds.of(1));

    assertAll(
        () ->
            assertTrue(
                intakeExtension.isExtenderAtPosition(),
                "Extender should be at position after extending"),
        () ->
            assertEquals(
                IntakeExtensionConstants.ExtenderPositions.IN.getAngle().in(Rotations),
                intakeExtension.getPosition().in(Rotations),
                ACCEPTABLE_ERROR_IN_ROTATIONS,
                "Extender should be close to the IN position, in Rotations"));
  }

  private void simulateRunLoop(Measure<? extends TimeUnit> duration) {
    Time now = Units.Seconds.of(0);
    Time target = now.plus(duration);
    Time period = Units.Seconds.of(Simulation.TIME_PERIOD);

    do {
      intakeExtension.periodic();

      SimHooks.stepTiming(Simulation.TIME_PERIOD);
      now = now.plus(period);
    } while (now.lt(target));
  }
}
