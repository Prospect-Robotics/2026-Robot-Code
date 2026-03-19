// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813;

import com.team2813.subsystems.drive.AllDrivetrains;
import com.team2813.subsystems.drive.AllTunerConstants;
import com.team2813.util.HubStatusUtil;
import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.nio.file.Files;
import java.nio.file.Path;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;
  private final Mode mode;

  private static final Path USB_LOG_PATH = Path.of("/U/logs");
  private static final Path ROBORIO_LOG_PATH = Path.of("/home/lvuser/logs");


  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    RobotController.setTimeSource(Logger::getTimestamp);
    // Set up data receivers & replay source
    mode = getCurrentModeFromEnv();
    System.out.printf("Current Mode: %s%n", mode);

    switch (mode) {
      case REAL:

        // Is the usb plugged in? If not, log to rio.
        Path pathToLog = Files.exists(USB_LOG_PATH) ? USB_LOG_PATH : ROBORIO_LOG_PATH;

        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter(pathToLog.toString()));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Get the constants for the current drivetrain and log them.
    // This call needs to be before the call to Logger.start().
    AllTunerConstants tunerConstants = AllDrivetrains.forRoboRIO();

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer(tunerConstants, mode);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (mode != Mode.REAL) {
      SimulationVisualizer.getInstance().periodic();
    }

    Logger.recordOutput("HubStatus/Our Hub Status", HubStatusUtil.isHubActive());

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    // This makes sure that the autonomous command isn't running after we exit the autonomous
    // period
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.stopEverything();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // Do not spam the logs with "Button x on port y not available" log messages.
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /** Gets the {@link Mode} that the robot should use. */
  static Mode getCurrentModeFromEnv() {
    return isReal() ? Mode.REAL : getSimMode();
  }

  /**
   * Gets the {@link Mode} that the robot should use in simulation. This method will never return
   * {@link Mode#REAL}, as that is never appropriate for robot simulation. If the gradle {@code
   * replayWatch} task is run, this will return {@link Mode#REPLAY} automatically.
   *
   * @return The {@link Mode} that the robot should use if it is being simulated.
   */
  private static Mode getSimMode() {
    // The environment variable "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE" is set to "true" when running
    // `replayWatch`. This will then only return `Mode.REPLAY` when we are in replay mode. Note that
    // this will set `Mode.REPLAY` if the user sets this environment variable, but that is probably
    // not going to happen due to the long, specific name, and if it does, that is their problem :3.
    if (Boolean.parseBoolean(System.getenv("FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE"))) {
      return Mode.REPLAY;
    } else {
      return Mode.SIM;
    }
  }
}
