package com.team2813;

import com.team2813.subsystems.drive.AllDrivetrains;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith(WPILibExtension.class)
public class RobotContainerTest {
  @Test
  public void canCreateRobotAndRunPeriodic(CommandTester tester) {
    Assumptions.assumeTrue(
        Constants.simMode == Constants.Mode.SIM, "The sim mode must be sim to run tests!");
    // create a robot container
    RobotContainer robotContainer = new RobotContainer(AllDrivetrains.drWomp());
    // Run one periodic cycle
    tester.runUntilComplete(Commands.none());
  }
}
