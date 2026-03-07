// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813;

import static com.team2813.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team2813.commands.DriveCommands;
import com.team2813.subsystems.climb.Climb;
import com.team2813.subsystems.climb.ClimbIO;
import com.team2813.subsystems.climb.ClimbIOReal;
import com.team2813.subsystems.climb.ClimbIOSim;
import com.team2813.subsystems.drive.AllTunerConstants;
import com.team2813.subsystems.drive.Drive;
import com.team2813.subsystems.drive.GyroIO;
import com.team2813.subsystems.drive.GyroIOPigeon2;
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
import com.team2813.subsystems.kicker.Kicker;
import com.team2813.subsystems.kicker.KickerIO;
import com.team2813.subsystems.kicker.KickerIOReal;
import com.team2813.subsystems.kicker.KickerIOSim;
import com.team2813.subsystems.shooter.*;
import com.team2813.subsystems.vision.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Mode mode;

  // Subsystems
  private final Drive drive;
  private final Hopper hopper;
  private final Vision vision;
  private final Climb climb;

  private final IntakeExtension intakeExtension;
  private final IntakeRoller intakeRoller;

  private final Shooter shooter;
  private final Kicker kicker;
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param tunerConstants The tuner constants for the robot.
   */
  public RobotContainer(AllTunerConstants tunerConstants, Mode mode) {
    this.mode = mode;
    switch (mode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                mode,
                tunerConstants,
                new GyroIOPigeon2(tunerConstants),
                new ModuleIOTalonFX(tunerConstants.frontLeft(), tunerConstants),
                new ModuleIOTalonFX(tunerConstants.frontRight(), tunerConstants),
                new ModuleIOTalonFX(tunerConstants.backLeft(), tunerConstants),
                new ModuleIOTalonFX(tunerConstants.backRight(), tunerConstants));

        hopper = new Hopper(new HopperIOReal());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                () -> {},
                new VisionIOPhotonVision(
                    VisionConstants.RED_BACK_LEFT_COLOR_CAMERA_NAME,
                    VisionConstants.RED_BACK_LEFT_CAM_FROM_ROBOT),
                new VisionIOPhotonVision(
                    VisionConstants.GREEN_BACK_RIGHT_COLOR_CAMERA_NAME,
                    VisionConstants.GREEN_BACK_RIGHT_CAM_FROM_ROBOT),
                new VisionIOPhotonVision(
                    VisionConstants.BLUE_FRONT_MONO_CAMERA_NAME,
                    VisionConstants.BLUE_FRONT_CAM_FROM_ROBOT));
        intakeExtension = new IntakeExtension(new IntakeExtensionIOReal());
        intakeRoller = new IntakeRoller(new IntakeRollerIOReal());

        shooter = new Shooter(new ShooterIOReal());
        kicker = new Kicker(new KickerIOReal());

        climb = new Climb(new ClimbIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                mode,
                tunerConstants,
                new GyroIO() {},
                new ModuleIOSim(tunerConstants.frontLeft()),
                new ModuleIOSim(tunerConstants.frontRight()),
                new ModuleIOSim(tunerConstants.backLeft()),
                new ModuleIOSim(tunerConstants.backRight()));

        hopper = new Hopper(new HopperIOSim());

        VisionSystemSim visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(APRIL_TAG_LAYOUT);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                () -> visionSim.update(drive.getPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.RED_BACK_LEFT_COLOR_CAMERA_NAME,
                    VisionConstants.RED_BACK_LEFT_CAM_FROM_ROBOT,
                    visionSim),
                new VisionIOPhotonVisionSim(
                    VisionConstants.GREEN_BACK_RIGHT_COLOR_CAMERA_NAME,
                    VisionConstants.GREEN_BACK_RIGHT_CAM_FROM_ROBOT,
                    visionSim),
                new VisionIOPhotonVisionSim(
                    VisionConstants.BLUE_FRONT_MONO_CAMERA_NAME,
                    VisionConstants.BLUE_FRONT_CAM_FROM_ROBOT,
                    visionSim));
        intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());
        intakeRoller = new IntakeRoller(new IntakeRollerIOSim());

        shooter = new Shooter(new ShooterIOSim());
        kicker = new Kicker(new KickerIOSim());

        climb = new Climb(new ClimbIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                mode,
                tunerConstants,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

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
        kicker = new Kicker(new KickerIO() {});

        climb = new Climb(new ClimbIO() {});
        break;
    }

    // Registers all named commands.
    namedCommandsRegistration();
    // Creates the autoBuilder, necessary for pathplanner, must be run after
    // namedCommandsRegistration because the registries freeze after.
    drive.initializeAutoBuilder();

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
    // Operator controls
    // Operator Intake roller Bindings
    operatorController.leftBumper().whileTrue(intakeRoller.outtakeCommand());

    // Intake Extension Bindings
    intakeExtension.setManualOverrideController(
        () -> MathUtil.applyDeadband(-operatorController.getLeftY(), 0.1));

    BooleanSupplier extensionInterruptionCondition =
        () ->
            (intakeExtension
                    .isExtenderAtPosition() // Either the intakeExtender reaches the setpoint.
                || Math.abs(operatorController.getLeftY())
                    > 0.3); // Or the operator interrupts by moving the left joystick left/right.

    operatorController
        .b()
        .whileTrue(
            new ParallelCommandGroup(intakeExtension.wallEMode(), intakeRoller.intakeCommand()));

    // Defensive Stop.
    operatorController.rightBumper().onTrue(new InstantCommand(drive::stopWithX));

    // Feeder controls
    operatorController.leftBumper().whileTrue(hopper.outtakeCommand());
    operatorController.povLeft().whileTrue(hopper.intakeCommand());

    // Operator intake roller bindings.
    operatorController.povRight().whileTrue(intakeRoller.intakeCommand());
    operatorController.leftTrigger().whileTrue(intakeRoller.outtakeCommand());

    // Spool shooter commands
    operatorController.rightTrigger().whileTrue(shooter.spoolShooterTrenchSpeedCommand());
    operatorController.x().whileTrue(shooter.spoolShooterHubSpeedCommand());

    // Driver controls
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // Driver intake roller bindings
    driveController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                intakeRoller.intakeCommand(),
                intakeExtension.extendCommand().until(extensionInterruptionCondition)));

    // Runs the Kicker Wheels toward the shooter.
    driveController.leftTrigger().whileTrue(kicker.shootCommand());

    // FIXME: Test climb bindings! Remove later!
    driveController.y().onTrue(climb.setInnerClimbPositionCommand(Climb.InnerClimbHeight.UP));
    driveController.a().onTrue(climb.setInnerClimbPositionCommand(Climb.InnerClimbHeight.DOWN));

    driveController.povLeft().onTrue(climb.setOuterClimbPositionCommand(Climb.OuterClimbHeight.UP));
    driveController
        .povRight()
        .onTrue(climb.setOuterClimbPositionCommand(Climb.OuterClimbHeight.DOWN));
    // END CLIMB BINDINGS.

    // hub shot command
    driveController
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                kicker.shootCommand(), hopper.intakeCommand(), intakeRoller.intakeCommand()));

    // Reset robot orientation, but keeps its position on the field.
    driveController
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Used for stopping the intake roller if the auto command ends prematurely.
   */
  public void stopIntakeRoller() {
    intakeRoller.stop();
  }

  private void namedCommandsRegistration() {
    NamedCommands.registerCommand(
        "TrenchShot",
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                shooter.spoolShooterTrenchSpeedCommand(),
                new SequentialCommandGroup(
                    new WaitUntilCommand(shooter::isMotorVelocityWithinTolerance),
                    new ParallelCommandGroup(kicker.shootCommand(), hopper.intakeCommand()))),
            new WaitCommand(3.5)));

    NamedCommands.registerCommand(
        "HubShot",
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                shooter.spoolShooterHubSpeedCommand(),
                new SequentialCommandGroup(
                    new WaitUntilCommand(shooter::isMotorVelocityWithinTolerance),
                    new ParallelCommandGroup(kicker.shootCommand(), hopper.intakeCommand()))),
            new WaitCommand(3.5)));

    NamedCommands.registerCommand(
        "ExtendIntake",
        intakeExtension.extendCommand().until(intakeExtension::isExtenderAtPosition));

    NamedCommands.registerCommand(
        "RetractIntake",
        intakeExtension.retractCommand().until(intakeExtension::isExtenderAtPosition));

    // Intake roller motor control.
    NamedCommands.registerCommand("SpinRollerIntake", new InstantCommand(intakeRoller::intake));

    NamedCommands.registerCommand("SpinRollerOuttake", new InstantCommand(intakeRoller::outtake));

    NamedCommands.registerCommand("StopRoller", new InstantCommand(intakeRoller::stop));

    NamedCommands.registerCommand("WalleMode", intakeExtension.wallEMode());
  }
}
