package com.team2813.subsystems.intakeroller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team2813.Constants;
import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

@InitWPILib
public class IntakeRollerTest {
  @BeforeAll
  public static void verifySim() {
    // verify that we're in sim mode before running tests
    assertTrue(
        Constants.simMode == Constants.Mode.SIM, "Must be in sim mode to run IntakeRoller tests");
  }

  @Test
  public void intakeRollerIntakeTest() {
    // create an intake roller subsystem
    IntakeRoller intakeRoller = new IntakeRoller(new IntakeRollerIOSim());

    // ensure hardware limits are disabled
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    // run the intake command for a few cycles

    for (int i = 0; i < 50; i++) {
      intakeRoller.intake();
      intakeRoller.periodic();
      SimHooks.stepTiming(Constants.SIM_TIME_PERIOD);
    }

    // verify that the motor output voltage is set to the intake voltage
    assertTrue(
        intakeRoller.getIntakeRollerVoltage() > 0,
        "Intake roller voltage should be positive for intake");
  }

  @Test
  public void intakeRollerOuttakeTest() {
    // create an intake roller subsystem
    IntakeRoller intakeRoller = new IntakeRoller(new IntakeRollerIOSim());

    // to ensure hardware limits on disabled mode disabled
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    // run the outtake command for a few cycles
    for (int i = 0; i < 50; i++) {
      intakeRoller.outtake();
      intakeRoller.periodic();
      SimHooks.stepTiming(Constants.SIM_TIME_PERIOD);
    }

    // verify that the motor output voltage is set to the outtake voltage
    assertTrue(
        intakeRoller.getIntakeRollerVoltage() < 0,
        "Intake roller voltage should be negative for outtake");
  }

  @Test
  public void intakeRollerStopTest() {
    // create an intake roller subsystem
    IntakeRoller intakeRoller = new IntakeRoller(new IntakeRollerIOSim());

    // ensure that hardware limits are disabled
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    // run the stop command for a few cycles
    intakeRoller.stop();
    for (int i = 0; i < 50; i++) {
      intakeRoller.periodic();
      SimHooks.stepTiming(Constants.SIM_TIME_PERIOD);
    }

    // verify that the motor output voltage is set to 0
    assertEquals(0, intakeRoller.getIntakeRollerVoltage(), 0.01);
  }
}
