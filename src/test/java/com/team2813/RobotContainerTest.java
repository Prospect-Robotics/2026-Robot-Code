package com.team2813;

import com.team2813.lib2813.testing.junit.jupiter.CommandTester;
import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import com.team2813.subsystems.drive.AllDrivetrains;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.Test;
import org.junitpioneer.jupiter.ClearEnvironmentVariable;

@InitWPILib
@ClearEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE")
public class RobotContainerTest {
  @Test
  public void canCreateRobotAndRunPeriodic(CommandTester tester) {
    // create a robot container
    RobotContainer robotContainer = new RobotContainer(AllDrivetrains.rebuiltDrivetrain());
    // Run one periodic cycle
    tester.runUntilComplete(Commands.none());
  }
}
