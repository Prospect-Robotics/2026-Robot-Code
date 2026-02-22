package com.team2813.util;

import static edu.wpi.first.units.Units.*;

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

  private static final SimulationVisualizer instance = new SimulationVisualizer();

  private SimulationVisualizer() {}

  public static SimulationVisualizer getInstance() {
    return instance;
  }

  /** The position of the intake extension. Default at 0 (fully retracted) */
  private Distance intakeExtensionPosition = Inches.of(0);

  /** The Mech2d Canvas to draw the intake on (Size of the robot in inches) */
  LoggedMechanism2d intakeExtensionCanvas = new LoggedMechanism2d(2, 1, new Color8Bit("#008cff"));

  /**
   * Root node of the intake extension mechanism, located at the pivot point of the intake (relative
   * to the robot center)
   */
  LoggedMechanismRoot2d intakeExtensionRoot =
      intakeExtensionCanvas.getRoot("Intake Extension", 
      1 + 0.34, // 1m is just the center of the mech2d canvas; 0.34m is the offset of hopper front from robot center
      0.1  // placeholder value for now: 0.1m off robot base.
      );

  /**
   * Ligament representing the intake extension, extending to the left from the root. Length is
   * updated in periodic to match the position of the intake extension.
   */
  LoggedMechanismLigament2d intakeExtensionLigament =
      intakeExtensionRoot.append(
          new LoggedMechanismLigament2d(
              "Intake Extension",
              intakeExtensionPosition.in(Meters),
              0,
              10.0,
              new Color8Bit("#ff9900")));

  /** Update the simulation visualizer with the current position of the intake extension. */
  public void periodic() {
    SmartDashboard.putData("Intake Extension Visualization", intakeExtensionCanvas);
    Logger.recordOutput("Intake Extension Visualization", intakeExtensionCanvas);
  }

  /**
   * Update the position of the intake extension in the simulation visualizer. The position is the
   * distance from the retracted position, so it should be 0 when the intake is fully retracted and
   * should increase as the intake extends.
   *
   * @param position - the distance from the retracted position of the intake extension, in any
   *     DistanceUnit.
   */
  public void updateIntakeExtensionPosition(Distance position) {
    intakeExtensionPosition = position;
    intakeExtensionLigament.setLength(intakeExtensionPosition.in(Meters));
  }
}
