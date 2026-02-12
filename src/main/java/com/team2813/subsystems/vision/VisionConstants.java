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
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static final String LEFT_COLOR_CAMERA_NAME = "left_color";
  public static final String RIGHT_COLOR_CAMERA_NAME = "right_color";
  public static final String MIDDLE_MONO_CAMERA_NAME = "middle_monochrome";

  // Robot to camera transforms
  public static final Transform3d ROBOT_TO_LEFT_CAM =
      new Transform3d(
          new Translation3d(Centimeters.of(31), Centimeters.of(21), Centimeters.of(20)),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-30)));

  public static final Transform3d ROBOT_TO_RIGHT_CAM =
      new Transform3d(
          new Translation3d(Centimeters.of(29), Centimeters.of(-23), Centimeters.of(13)),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(20)));

  public static final Transform3d ROBOT_TO_MID_CAM =
      new Transform3d(
          // Field of translation in X,Y,Z (X: front-back (+:front)), (Y: left-right (+:left), (Z:
          // up-down (+:up))
          // Rotations: Roll, pitch, yaw (applied in that order.
          new Translation3d(Inches.of(15), Inches.of(0), Inches.of(3.25)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(0)));

  // Basic filtering thresholds
  public static final double MAX_AMBIGUITY = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] CAMERA_STD_DEV_FACTORS =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
  public static final double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
      Double.POSITIVE_INFINITY; // No rotation data available
}
