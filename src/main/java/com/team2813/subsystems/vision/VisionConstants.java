// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String LEFT_COLOR_CAMERA_NAME = "left_color";
  public static String RIGHT_COLOR_CAMERA_NAME = "right_color";
  public static String MIDDLE_MONO_CAMERA_NAME = "middle_monochrome";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d ROBOT_TO_LEFT_CAM =
      new Transform3d(
          new Translation3d(Centimeters.of(31), Centimeters.of(21), Centimeters.of(20)),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-30)));

  public static Transform3d ROBOT_TO_RIGHT_CAM =
      new Transform3d(
          new Translation3d(Centimeters.of(29), Centimeters.of(-23), Centimeters.of(13)),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(20)));

  public static Transform3d ROBOT_TO_MID_CAM =
      new Transform3d(
          // Field of translation in X,Y,Z (X: front-back (+:front)), (Y: left-right (+:left), (Z:
          // up-down (+:up))
          // Rotations: Roll, pitch, yaw (applied in that order.
          new Translation3d(Inches.of(15), Inches.of(0), Inches.of(3.25)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
