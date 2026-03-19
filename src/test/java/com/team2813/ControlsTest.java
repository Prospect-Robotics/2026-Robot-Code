package com.team2813;

import static com.google.common.truth.Truth.assertThat;
import static com.team2813.lib2813.testing.truth.Pose2dSubject.assertThat;

import com.team2813.lib2813.testing.junit.jupiter.CommandTester;
import com.team2813.lib2813.testing.junit.jupiter.InitWPILib;
import com.team2813.subsystems.drive.AllDrivetrains;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.junitpioneer.jupiter.ClearEnvironmentVariable;

@InitWPILib
@ClearEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE")
public class ControlsTest {
  @ParameterizedTest
  @MethodSource("redAlliance")
  public void driveTestRed(AllianceStationID allianceStationID, CommandTester commandTester) {
    AllianceStationID origId = DriverStation.getRawAllianceStation();

    // Prepare
    DriverStationSim.setAllianceStationId(allianceStationID);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    XboxControllerSim driverSim = new XboxControllerSim(0);

    try {
      RobotContainer robotContainer =
          new RobotContainer(AllDrivetrains.rebuiltDrivetrain(), Mode.SIM);

      // Act (Forwards)
      driverSim.setLeftX(0);
      driverSim.setLeftY(1);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(Commands.none());
      Pose2d startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      Pose2d endPose = robotContainer.drive.getPose();

      // Assert (Forwards)
      assertThat(endPose).translation().x().isGreaterThan(startPose.getX());
      assertThat(endPose).translation().y().isWithin(0.1).of(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());

      // Act (Left)
      driverSim.setLeftX(1);
      driverSim.setLeftY(0);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(new WaitCommand(1));
      startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      endPose = robotContainer.drive.getPose();

      // Assert (Left)
      assertThat(endPose).translation().x().isWithin(0.1).of(startPose.getX());
      assertThat(endPose).translation().y().isGreaterThan(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());

      // Act (Backwards)
      driverSim.setLeftX(0);
      driverSim.setLeftY(-1);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(new WaitCommand(1));
      startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      endPose = robotContainer.drive.getPose();

      // Assert (Backwards)
      assertThat(endPose).translation().x().isLessThan(startPose.getX());
      assertThat(endPose).translation().y().isWithin(0.1).of(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());

      // Act (Right)
      driverSim.setLeftX(-1);
      driverSim.setLeftY(0);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(new WaitCommand(1));
      startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      endPose = robotContainer.drive.getPose();

      // Assert (Right)
      assertThat(endPose).translation().x().isWithin(0.1).of(startPose.getX());
      assertThat(endPose).translation().y().isLessThan(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());
    } finally {
      DriverStationSim.setAllianceStationId(origId);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();

      driverSim.setLeftX(0);
      driverSim.setLeftY(0);
      driverSim.notifyNewData();
    }
  }

  @ParameterizedTest
  @MethodSource("blueAlliance")
  public void driveTestBlue(AllianceStationID allianceStationID, CommandTester commandTester) {
    AllianceStationID origId = DriverStation.getRawAllianceStation();

    // Prepare
    DriverStationSim.setAllianceStationId(allianceStationID);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    XboxControllerSim driverSim = new XboxControllerSim(0);

    try {
      RobotContainer robotContainer =
          new RobotContainer(AllDrivetrains.rebuiltDrivetrain(), Mode.SIM);
      robotContainer.drive.setPose(new Pose2d(Translation2d.kZero, Rotation2d.k180deg));

      // Act (Forwards)
      driverSim.setLeftX(0);
      driverSim.setLeftY(1);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(Commands.none());
      Pose2d startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      Pose2d endPose = robotContainer.drive.getPose();

      // Assert (Forwards)
      assertThat(endPose).translation().x().isLessThan(startPose.getX());
      assertThat(endPose).translation().y().isWithin(0.1).of(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());

      // Act (Left)
      driverSim.setLeftX(1);
      driverSim.setLeftY(0);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(new WaitCommand(1));
      startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      endPose = robotContainer.drive.getPose();

      // Assert (Left)
      assertThat(endPose).translation().x().isWithin(0.1).of(startPose.getX());
      assertThat(endPose).translation().y().isLessThan(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());

      // Act (Backwards)
      driverSim.setLeftX(0);
      driverSim.setLeftY(-1);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(new WaitCommand(1));
      startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      endPose = robotContainer.drive.getPose();

      // Assert (Backwards)
      assertThat(endPose).translation().x().isGreaterThan(startPose.getX());
      assertThat(endPose).translation().y().isWithin(0.1).of(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());

      // Act (Right)
      driverSim.setLeftX(-1);
      driverSim.setLeftY(0);
      driverSim.notifyNewData();
      commandTester.runUntilComplete(new WaitCommand(1));
      startPose = robotContainer.drive.getPose();
      commandTester.runUntilComplete(new WaitCommand(1));
      endPose = robotContainer.drive.getPose();

      // Assert (Right)
      assertThat(endPose).translation().x().isWithin(0.1).of(startPose.getX());
      assertThat(endPose).translation().y().isGreaterThan(startPose.getY());
      assertThat(endPose).rotation().isWithin(0.1).of(startPose.getRotation());
    } finally {
      DriverStationSim.setAllianceStationId(origId);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();

      driverSim.setLeftX(0);
      driverSim.setLeftY(0);
      driverSim.notifyNewData();
    }
  }

  public static AllianceStationID[] redAlliance() {
    return new AllianceStationID[] {
      AllianceStationID.Red1, AllianceStationID.Red2, AllianceStationID.Red3
    };
  }

  public static AllianceStationID[] blueAlliance() {
    return new AllianceStationID[] {
      AllianceStationID.Blue1, AllianceStationID.Blue2, AllianceStationID.Blue3
    };
  }
}
