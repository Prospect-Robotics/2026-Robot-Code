package com.team2813;

import com.team2813.lib2813.testing.junit.jupiter.CommandTester;
import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import com.team2813.subsystems.drive.AllDrivetrains;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.Test;

@InitWPILib
public class RobotContainerTest {
  @Test
  public void canCreateRobotAndRunPeriodic(CommandTester tester) {
    // create a robot container
    RobotContainer robotContainer = new RobotContainer(AllDrivetrains.drWomp());
    // Run one periodic cycle
    tester.runUntilComplete(Commands.none());
  }
}
