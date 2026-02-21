package com.team2813.subsystems.fuelprocessingsim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

/** TODO(vdikov): Write description */
public class FuelProcessingSim extends SubsystemBase {
  private static final int HOPPER_FUEL_CAPACITY = 46;
  // See
  // https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/#over-the-bumper-otb-intakes
  // for description on how this intake simulation is set up.
  private final IntakeSimulation intakeSimulation;
  private final AbstractDriveTrainSimulation driveTrainSimulation;

  public FuelProcessingSim(AbstractDriveTrainSimulation driveTrainSimulation) {
    this.driveTrainSimulation = driveTrainSimulation;
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrainSimulation,
            // Width of the intake
            Meters.of(0.7), // TODO(vdikov): Confirm in CAD
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2), // TODO(vdikov): Confirm in CAD
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 1 note
            HOPPER_FUEL_CAPACITY);
    this.intakeSimulation.addGamePiecesToIntake(
        10); // Start with 10 fuel in the intake for testing purposes
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "FuelProcessingSim/gamePiecesAmount", intakeSimulation.getGamePiecesAmount());
  }

  /** TODO(vdikov): Write description. */
  public void runFuelIntake(boolean intakeActivated) {
    if (intakeActivated) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  /**
   * TODO(vdikov): Write description. based on
   * https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/
   */
  public void launchFuel(AngularVelocity flywheelAngularVelocity) {

    if (flywheelAngularVelocity.lt(RotationsPerSecond.of(5))) return;
    if (!intakeSimulation.obtainGamePieceFromIntake()) return;

    var robotSimulationWorldPose = driveTrainSimulation.getSimulatedDriveTrainPose();
    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            // Specify the position of the chassis when the note is launched
            // robotSimulationWorldPose.getTranslation(),
            robotSimulationWorldPose.getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, 0),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
            // the projectile
            driveTrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // The shooter facing direction is the same as the robot’s facing direction
            robotSimulationWorldPose.getRotation(),
            // Add the shooter’s rotation
            // + shooterRotation,  // We don't have a turret shooter.
            // Initial height of the flying note
            Meters.of(0.45),
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000
            // RPM
            // MetersPerSecond.of(flywheelAngularVelocity.in(RotationsPerSecond) / 5),
            MetersPerSecond.of(5),
            // The angle at which the note is launched
            Degrees.of(55));
    fuelOnFly
        // Set the target center to the Rebbuilt Hub of the current alliance
        .withTargetPosition(
            () ->
                FieldMirroringUtils.toCurrentAllianceTranslation(
                    new Translation3d(0.25, 5.56, 2.3)))
        // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the speaker's
        // "mouth")
        .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
        // Set a callback to run when the fuel hits the target
        .withHitTargetCallBack(() -> System.out.println("Hit hub, +1 point!"));
    fuelOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
        // Callback for when the fuel will eventually hit the target (if configured)
        (pose3ds) ->
            Logger.recordOutput(
                "Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
        // Callback for when the fuel will eventually miss the target, or if no target is configured
        (pose3ds) ->
            Logger.recordOutput(
                "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));
    fuelOnFly
        // Configure the note projectile to become a NoteOnField upon touching the ground
        .enableBecomesGamePieceOnFieldAfterTouchGround();

    // Add the projectile to the simulated arena
    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
  }
}
