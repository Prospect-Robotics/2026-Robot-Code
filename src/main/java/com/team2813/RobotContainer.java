// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2813.commands.DriveCommands;
import com.team2813.subsystems.drive.AllTunerConstants;
import com.team2813.subsystems.drive.Drive;
import com.team2813.subsystems.drive.GyroIO;
import com.team2813.subsystems.drive.GyroIOPigeon2;
import com.team2813.subsystems.drive.ModuleIO;
import com.team2813.subsystems.drive.ModuleIOSim;
import com.team2813.subsystems.drive.ModuleIOTalonFX;
import com.team2813.subsystems.hopper.*;
import com.team2813.subsystems.shooter.Shooter;
import com.team2813.subsystems.shooter.ShooterIO;
import com.team2813.subsystems.shooter.ShooterIOReal;
import com.team2813.subsystems.shooter.ShooterIOSim;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Hopper hopper;
  private final Shooter shooter;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new ModuleIOTalonFX(tunerConstants.backRight(), tunerConstants));

        hopper = new Hopper(new HopperIOReal());

        shooter = new Shooter(new ShooterIOReal());
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(tunerConstants.frontLeft()),
        // new ModuleIOTalonFXS(tunerConstants.frontRight()),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(tunerConstants.backRight()));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                tunerConstants,
                new GyroIO() {},
                new ModuleIOSim(tunerConstants.frontLeft()),
                new ModuleIOSim(tunerConstants.frontRight()),
                new ModuleIOSim(tunerConstants.backLeft()),
                new ModuleIOSim(tunerConstants.backRight()));

        hopper = new Hopper(new HopperIOSim());

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
                new ModuleIO() {});

        hopper = new Hopper(new HopperIO() {});

        shooter = new Shooter(new ShooterIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive, tunerConstants));
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

    // Configure the button bindings
    configureButtonBindings(tunerConstants);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(AllTunerConstants tunerConstants) {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.leftBumper().onTrue(hopper.intakeCommand()).onFalse(hopper.stopCommand());
    controller.rightBumper().onTrue(hopper.outtakeCommand()).onFalse(hopper.stopCommand());

    controller.leftTrigger().onTrue(shooter.intakeCommand()).onFalse(shooter.stopCommand());
    controller.rightTrigger().onTrue(shooter.outakeCommand()).onFalse(shooter.stopCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
