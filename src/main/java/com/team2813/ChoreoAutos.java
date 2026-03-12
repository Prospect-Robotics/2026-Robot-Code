package com.team2813;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team2813.subsystems.drive.Drive;
import com.team2813.subsystems.hopper.Hopper;
import com.team2813.subsystems.intakeextension.IntakeExtension;
import com.team2813.subsystems.intakeroller.IntakeRoller;
import com.team2813.subsystems.kicker.Kicker;
import com.team2813.subsystems.shooter.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Objects;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class ChoreoAutos implements AutoSelector {
  private final AutoFactory autoFactory;
  private final SelectorWrapper autoChooser = new SelectorWrapper("Choreo Auto Selector");
  private final IntakeRoller intakeRoller;
  private final IntakeExtension intakeExtension;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Hopper hopper;

  ChoreoAutos(
      Drive drive,
      IntakeRoller intakeRoller,
      IntakeExtension intakeExtension,
      Shooter shooter,
      Kicker kicker,
      Hopper hopper,
      Runnable registerNamedCommands) {
    this(
        shouldAddPathPlannerAutos(),
        drive,
        intakeRoller,
        intakeExtension,
        shooter,
        kicker,
        hopper,
        registerNamedCommands);
  }

  private static boolean shouldAddPathPlannerAutos() {
    Preferences.initBoolean("Choreo/copy-pathplanner", true);
    return Preferences.getBoolean("Choreo/copy-pathplanner", true);
  }

  private ChoreoAutos(
      boolean addPathPlannerAutos,
      Drive drive,
      IntakeRoller intakeRoller,
      IntakeExtension intakeExtension,
      Shooter shooter,
      Kicker kicker,
      Hopper hopper,
      Runnable registerNamedCommands) {
    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            (SwerveSample sample) -> followTrajectory(drive, sample),
            true,
            drive,
            this::logTrajectory);
    if (addPathPlannerAutos) {
      registerNamedCommands.run();
      drive.initializeAutoBuilder();
      // iterate over all auto names, and add each one to the auto chooser
      for (String name : AutoBuilder.getAllAutoNames()) {
        autoChooser.addCmd(name, () -> AutoBuilder.buildAuto(name));
      }
    }
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    this.intakeRoller = intakeRoller;
    this.intakeExtension = intakeExtension;
    this.shooter = shooter;
    this.kicker = kicker;
    this.hopper = hopper;

    autoChooser.addRoutine("CHOR - Neutral Zone Auto", this::neutralZoneAuto);
  }

  private AutoRoutine neutralZoneAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Shoot Neutral Zone Fuel From Trench");

    AutoTrajectory goToNeutral = routine.trajectory("right_neutral_fuel", 0);
    AutoTrajectory pickUpFuel = routine.trajectory("right_neutral_fuel", 1);
    AutoTrajectory goToTrench = routine.trajectory("right_neutral_fuel", 2);

    routine.active().onTrue(Commands.sequence(goToNeutral.resetOdometry(), goToNeutral.cmd()));

    pickUpFuel
        .active()
        .whileTrue(
            Commands.parallel(intakeExtension.extendCommand(), intakeRoller.intakeCommand()));

    goToNeutral.done().onTrue(pickUpFuel.cmd());
    pickUpFuel.done().onTrue(goToTrench.cmd());
    goToTrench.atTime("Spool").onTrue(shooter.spoolShooterTrenchSpeedCommand());
    goToTrench
        .done()
        .onTrue(
            Commands.sequence(
                    new WaitUntilCommand(shooter::isMotorVelocityWithinTolerance),
                    Commands.parallel(kicker.shootCommand(), hopper.intakeCommand())
                        .withTimeout(10) // TODO: Find out how long it takes to empty our hopper
                    )
                .finallyDo(shooter::stop));

    return routine;
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  @Override
  public void addOption(String name, Supplier<Command> generator) {
    autoChooser.addCmd(name, generator);
  }

  private final PIDController xController = new PIDController(5.0, 0, 0);
  private final PIDController yController = new PIDController(5.0, 0, 0);
  private final PIDController rotController = new PIDController(5.0, 0, 0);

  private void followTrajectory(Drive drive, SwerveSample sample) {
    ChassisSpeeds speeds = sample.getChassisSpeeds();
    Pose2d pose = drive.getPose();
    speeds.vxMetersPerSecond += xController.calculate(pose.getX(), sample.x);
    speeds.vyMetersPerSecond += yController.calculate(pose.getY(), sample.y);
    speeds.omegaRadiansPerSecond +=
        rotController.calculate(pose.getRotation().getRadians(), sample.heading);
    // need to convert to robot-relative
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation()));
    Logger.recordOutput("Odometry/TrajectorySetpoint", sample.getPose());
  }

  private void logTrajectory(Trajectory<SwerveSample> trajectory, boolean starting) {
    if (starting) {
      Trajectory<SwerveSample> traj = trajectory;
      if (DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false)) {
        traj = trajectory.flipped();
      }
      Logger.recordOutput("Odometry/Trajectory", traj.getPoses());
    } else {
      Logger.recordOutput("Odometry/Trajectory", new Pose2d[0]);
    }
  }

  private static final class SelectorWrapper extends LoggedNetworkInput {
    private String selected;
    private String wasSelected;
    private final String key;
    private final AutoChooser choreoChooser;
    private final SendableChooser<String> autoChooser;

    public SelectorWrapper(String key) {
      this.key = key;
      this.choreoChooser = new AutoChooser();
      this.autoChooser = new SendableChooser<>();
      SmartDashboard.putData(key, autoChooser);
      periodic();
      Logger.registerDashboardInput(this);
    }

    public void addCmd(String name, Supplier<Command> cmd) {
      autoChooser.addOption(name, name);
      choreoChooser.addCmd(key, cmd);
    }

    public void addRoutine(String name, Supplier<AutoRoutine> routine) {
      autoChooser.addOption(name, name);
      choreoChooser.addRoutine(name, routine);
    }

    private final LoggableInputs inputs =
        new LoggableInputs() {
          @Override
          public void toLog(LogTable table) {
            table.put(key, selected);
          }

          @Override
          public void fromLog(LogTable table) {
            selected = table.get(key, selected);
          }
        };

    @Override
    public void periodic() {
      if (!Logger.hasReplaySource()) {
        selected = autoChooser.getSelected();
      }
      Logger.processInputs(prefix + "/SmartDashboard", inputs);
      if (!Objects.equals(wasSelected, selected)) {
        choreoChooser.select(selected);
        wasSelected = selected;
      }
    }

    public Command selectedCommand() {
      return choreoChooser.selectedCommand();
    }
  }
}
