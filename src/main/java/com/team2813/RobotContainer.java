// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813;

import static com.team2813.Constants.onRed;
import static com.team2813.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;
import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2813.commands.DriveCommands;
import com.team2813.commands.IntakeExtensionDefaultCommand;
import com.team2813.subsystems.drive.AllTunerConstants;
import com.team2813.subsystems.drive.Drive;
import com.team2813.subsystems.drive.GyroIO;
import com.team2813.subsystems.drive.GyroIOPigeon2;
import com.team2813.subsystems.drive.GyroIOSim;
import com.team2813.subsystems.drive.ModuleIO;
import com.team2813.subsystems.drive.ModuleIOSim;
import com.team2813.subsystems.drive.ModuleIOTalonFX;
import com.team2813.subsystems.hopper.*;
import com.team2813.subsystems.intakeextension.IntakeExtension;
import com.team2813.subsystems.intakeextension.IntakeExtensionIO;
import com.team2813.subsystems.intakeextension.IntakeExtensionIOReal;
import com.team2813.subsystems.intakeextension.IntakeExtensionIOSim;
import com.team2813.subsystems.intakeroller.IntakeRoller;
import com.team2813.subsystems.intakeroller.IntakeRollerIO;
import com.team2813.subsystems.intakeroller.IntakeRollerIOReal;
import com.team2813.subsystems.intakeroller.IntakeRollerIOSim;
import com.team2813.subsystems.shooter.Shooter;
import com.team2813.subsystems.shooter.ShooterIO;
import com.team2813.subsystems.shooter.ShooterIOReal;
import com.team2813.subsystems.shooter.ShooterIOSim;
import com.team2813.subsystems.vision.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;

  private final Hopper hopper;
  private final Vision vision;

  private final IntakeExtension intakeExtension;
  private final IntakeRoller intakeRoller;

  private final Shooter shooter;
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static final Pose2d BLUE_HUB_POSITION = new Pose2d(4.580, 4.000, Rotation2d.kZero);
  private static final Pose2d RED_HUB_POSITION = new Pose2d(11.812, 4.000, Rotation2d.kZero);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param tunerConstants The tuner constants for the robot.
   */
  public RobotContainer(AllTunerConstants tunerConstants) {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                tunerConstants,
                new GyroIOPigeon2(tunerConstants),
                new ModuleIOTalonFX(tunerConstants.frontLeft(), tunerConstants),
                new ModuleIOTalonFX(tunerConstants.frontRight(), tunerConstants),
                new ModuleIOTalonFX(tunerConstants.backLeft(), tunerConstants),
                new ModuleIOTalonFX(tunerConstants.backRight(), tunerConstants),
                (robotPose) -> {});

        hopper = new Hopper(new HopperIOReal());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                () -> {},
                new VisionIOPhotonVision(
                    VisionConstants.LEFT_COLOR_CAMERA_NAME, VisionConstants.ROBOT_TO_LEFT_CAM),
                new VisionIOPhotonVision(
                    VisionConstants.RIGHT_COLOR_CAMERA_NAME, VisionConstants.ROBOT_TO_RIGHT_CAM),
                new VisionIOPhotonVision(
                    VisionConstants.MIDDLE_MONO_CAMERA_NAME, VisionConstants.ROBOT_TO_MID_CAM));
        intakeExtension = new IntakeExtension(new IntakeExtensionIOReal());
        intakeRoller = new IntakeRoller(new IntakeRollerIOReal());

        shooter = new Shooter(new ShooterIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.createMapleSimConfig(tunerConstants), new Pose2d(12, 2, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                tunerConstants,
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(
                    driveSimulation.getModules()[0],
                    Amps.of(tunerConstants.frontLeft().SlipCurrent)),
                new ModuleIOSim(
                    driveSimulation.getModules()[1],
                    Amps.of(tunerConstants.frontRight().SlipCurrent)),
                new ModuleIOSim(
                    driveSimulation.getModules()[2],
                    Amps.of(tunerConstants.backLeft().SlipCurrent)),
                new ModuleIOSim(
                    driveSimulation.getModules()[3],
                    Amps.of(tunerConstants.backRight().SlipCurrent)),
                driveSimulation::setSimulationWorldPose);

        hopper = new Hopper(new HopperIOSim());

        VisionSystemSim visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(APRIL_TAG_LAYOUT);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                () -> visionSim.update(drive.getPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.LEFT_COLOR_CAMERA_NAME,
                    VisionConstants.ROBOT_TO_LEFT_CAM,
                    drive::getPose,
                    visionSim),
                new VisionIOPhotonVisionSim(
                    VisionConstants.RIGHT_COLOR_CAMERA_NAME,
                    VisionConstants.ROBOT_TO_RIGHT_CAM,
                    drive::getPose,
                    visionSim),
                new VisionIOPhotonVisionSim(
                    VisionConstants.MIDDLE_MONO_CAMERA_NAME,
                    VisionConstants.ROBOT_TO_MID_CAM,
                    drive::getPose,
                    visionSim));
        intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());
        intakeRoller = new IntakeRoller(new IntakeRollerIOSim(driveSimulation));

        shooter = new Shooter(new ShooterIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                tunerConstants,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});

        hopper = new Hopper(new HopperIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                () -> {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        intakeExtension = new IntakeExtension(new IntakeExtensionIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});

        shooter = new Shooter(new ShooterIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Shooter SysID Routine", shooter.sysIDRoutine());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive commands
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    driveController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                this::getBotToHub));

    // Reset robot orientation, but keeps its position on the field.
    driveController
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                }));

    // Feeder and Vectoring Bindings
    driveController.leftBumper().onTrue(hopper.intakeCommand()).onFalse(hopper.stopCommand());
    driveController.rightBumper().onTrue(hopper.outtakeCommand()).onFalse(hopper.stopCommand());

    // Shooter Bindings
    driveController.leftTrigger().whileTrue(shooter.intakeCommand());
    driveController.rightTrigger().whileTrue(shooter.outakeCommand());

    // Intake Roller Bindings
    driveController.rightBumper().whileTrue(intakeRoller.intakeCommand());
    operatorController.leftTrigger().whileTrue(intakeRoller.outtakeCommand());

    // Intake Extension Bindings
    intakeExtension.setDefaultCommand(
        new IntakeExtensionDefaultCommand(intakeExtension, () -> -operatorController.getLeftY()));

    BooleanSupplier extensionInterruptionCondition =
        () ->
            (intakeExtension
                    .isExtenderAtPosition() // Either the intakeExtender reaches the setpoint.
                || Math.abs(operatorController.getLeftY())
                    > 0.3); // Or the operator interrupts by moving the left joystick left/right.

    operatorController
        .a()
        .onTrue(
            (new StartEndCommand(
                    intakeExtension::extend, intakeExtension::stopMotor, intakeExtension))
                .until(extensionInterruptionCondition));

    operatorController
        .b()
        .onTrue(
            (new StartEndCommand(
                    intakeExtension::retract, intakeExtension::stopMotor, intakeExtension))
                .until(extensionInterruptionCondition));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Rotation2d getBotToHub() {
    Pose2d hub;
    if (onRed()) {
      hub = RED_HUB_POSITION;
    } else {
      hub = BLUE_HUB_POSITION;
    }
    return hub.getTranslation().minus(drive.getPose().getTranslation()).getAngle();
  }

  /**
   * Resets the simulation.
   *
   * <p>Borrowed from
   * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java#L394.
   */
  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(12, 2, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    // AlgaeHandler.getInstance().reset();
  }

  /**
   * Updates Simulated Arena; to be called from Robot.simulationPeriodic()
   *
   * <p>Borrowed from
   * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java#L402
   */
  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    // The pose by maplesim, including collisions with the field.
    // See https://www.chiefdelphi.com/t/simulated-robot-goes-through-walls-with-maplesim/508663.
    Logger.recordOutput(
        "FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    // Logger.recordOutput(
    //         "FieldSimulation/Staged Algae", AlgaeHandler.getInstance().periodic());
  }
}
