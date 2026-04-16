package com.team2813.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Generic class for visualizing the robot in simulation.
 *
 * <p>This class is meant to be used for visualizing the intake extension mechanism in the intake
 * extension simulation, but it can be extended to visualize other mechanisms as well.
 *
 * <p>The visualization is done using the Mechanism2d class from WPILib, which allows for drawing
 * simple shapes to represent the robot and its mechanisms.
 *
 * <p>The SimulationVisualizer class is a singleton, so it can be accessed from anywhere in the code
 * to update the visualization with the current state of the robot.
 */
public class SimulationVisualizer {

  private static final Angle INDEXER_PITCH_ANGLE =
      Degrees.of(4.75); // Pitch down relative to y axis.
  private static final SimulationVisualizer instance = new SimulationVisualizer();

  // Note: We might make this public if someone wants unit tests on this.
  private SimulationVisualizer() {}

  /** Returns the default instance of SimulationVisualizer. */
  public static SimulationVisualizer getInstance() {
    return instance;
  }

  /**
   * The position of the intake extension. Default at 0 inches (fully retracted). This value is used
   * to update the 3D model, if in use.
   */
  private Distance intakeExtensionPosition = Inches.of(0);

  /**
   * The angle of the shooter hood. Defaults to 0 rotations (horizontal, pointing east).
   * This value is used for both updating the 3D model.
   */
  private Angle shooterHoodAngle = Rotations.of(0);

  /** The Mech2d Canvas to draw the intake on (size of the robot in meters). */
  private LoggedMechanism2d intakeExtensionCanvas =
      new LoggedMechanism2d(2, 1, new Color8Bit("#008cff"));

  /**
   * The Mech2d Canvas to draw the angle of the shooter hood on (units of the Canvas are in meters).
   */
  private LoggedMechanism2d shooterHoodCanvas =
      new LoggedMechanism2d(2, 1, new Color8Bit("#008ccf"));

  /**
   * Root node of the intake extension mechanism, located at the pivot point of the intake (relative
   * to the robot center)
   */
  private LoggedMechanismRoot2d intakeExtensionRoot =
      intakeExtensionCanvas.getRoot(
          "Intake Extension",
          1 + 0.34, // 1m is just the center of the mech2d canvas; 0.34m is the offset of hopper
          // front from robot center
          0.1 // placeholder value for now: 0.1m off robot base.
          );

  /**
   * Root node of the hood mechanism.
   */
  private LoggedMechanismRoot2d shooterHoodRoot =
      intakeExtensionCanvas.getRoot("Shooter Hood",
          1, // Arbitrary values to make the ligament visible.
          0.5);

  /**
   * Ligament representing the intake extension, extending to the left from the root. Length is
   * updated in periodic to match the position of the intake extension.
   */
  private LoggedMechanismLigament2d intakeExtensionLigament =
      intakeExtensionRoot.append(
          new LoggedMechanismLigament2d(
              "Intake Extension",
              intakeExtensionPosition.in(Meters),
              0,
              10.0,
              new Color8Bit("#ff9900")));

  /**
   * Ligament representing the shooter hood.
   * The angle of the ligament is updated to match the angle of the hood.
   */
  private LoggedMechanismLigament2d shooterHoodLigament =
      intakeExtensionRoot.append(
          new LoggedMechanismLigament2d(
              "Shooter Hood",
              0.3,
              shooterHoodAngle.in(Degrees),
              10.0,
              new Color8Bit("#ff9900")
          )
      );

  /** Update the simulation visualizer with the current position of the intake extension. */
  public void periodic() {
    SmartDashboard.putData("SimulationVisualizer/Intake Extension Visualization", intakeExtensionCanvas);
    Logger.recordOutput("SimulationVisualizer/Intake Extension Visualization", intakeExtensionCanvas);

    SmartDashboard.putData("SimulationVisualizer/Shooter Hood Visualization", intakeExtensionCanvas);
    Logger.recordOutput("SimulationVisualizer/Shooter Hood Visualization", intakeExtensionCanvas);

    double intakeExtensionX =
        intakeExtensionPosition.in(Meters) * Math.cos(INDEXER_PITCH_ANGLE.in(Radians));
    double intakeExtensionZ =
        -intakeExtensionPosition.in(Meters) * Math.sin(INDEXER_PITCH_ANGLE.in(Radians));

    // Component Simulation for the 3D robot.
    Logger.recordOutput(
        "Component Positions",
        new Pose3d[] {
          // Hopper and indexer
          new Pose3d(intakeExtensionX, 0, intakeExtensionZ, new Rotation3d(0, 0, 0)),
        });
  }

  /**
   * Update the position of the intake extension in the simulation visualizer. The position is the
   * distance from the retracted position, so it should be 0 when the intake is fully retracted and
   * should increase as the intake extends.
   *
   * @param position The distance from the retracted position of the intake extension, in any {@link
   *     edu.wpi.first.units.DistanceUnit}.
   */
  public void updateIntakeExtensionPosition(Distance position) {
    intakeExtensionPosition = position;
    intakeExtensionLigament.setLength(intakeExtensionPosition.in(Meters));
  }

  /**
   * Update the rotation of the shooter hood in the simulation visualizer.
   * @param angle Angle from the starting position.
   */
  public void updateShooterHoodAngle(Angle angle) {
    shooterHoodAngle = angle;
    shooterHoodLigament.setAngle(angle.in(Degrees));
  }
}
