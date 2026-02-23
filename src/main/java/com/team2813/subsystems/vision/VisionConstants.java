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
  // See documentation/images/camera_position.png for a diagram of the camera position layout on the
  // robot.
  public static final String RED_BACK_LEFT_COLOR_CAMERA_NAME = "red_back_left_color";
  public static final String GREEN_BACK_RIGHT_COLOR_CAMERA_NAME = "green_back_right_color";
  public static final String BLUE_FRONT_MONO_CAMERA_NAME = "blue_front_monochrome";

  // Robot to camera transforms
  //
  // Translation in X,Y,Z (X: front-back (+:front)), (Y: left-right (+:left), (Z: up-down (+:up))
  // Rotations: rotations are defined around {x, y, z} axes in that order, which corresponds to
  // {roll, pitch, yaw} respectively.
  //
  // Here're some shorthand reminders:
  //  - we want to never have a roll in the camera, since that confuses all other math.
  //  - pitching the camera up by 15 degrees is represented with negative 15 degrees (-15) in the
  //    y/pitch rotation component.
  //  - turning the camera left by 20 degrees is represented with negative 20 degrees (-20) in the
  //    z/yaw rotation component.
  //  - turning the camera right by 20 degrees is represented with positive 20 degrees (+20) in the
  //    z/yaw rotation component.
  //
  // For further details, please consult with
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system
  // or
  // https://docs.photonvision.org/en/v2026.2.2/docs/apriltag-pipelines/coordinate-systems.html#camera-coordinate-frame for reference to the robot coordinate system.
  // Both referneces define the same coordinate convention, so use whichever is easier to understand
  // for you.
  public static final Transform3d RED_BACK_LEFT_CAM_FROM_ROBOT =
      new Transform3d(
          new Translation3d(Centimeters.of(-26), Centimeters.of(26), Centimeters.of(23)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(-160)));

  public static final Transform3d GREEN_BACK_RIGHT_CAM_FROM_ROBOT =
      new Transform3d(
          new Translation3d(Centimeters.of(-26), Centimeters.of(-26), Centimeters.of(23)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(-90)));

  public static final Transform3d BLUE_FRONT_CAM_FROM_ROBOT =
      new Transform3d(
          new Translation3d(Centimeters.of(-5.5), Centimeters.of(0), Centimeter.of(50.1)),
          new Rotation3d(Degrees.of(0), Degrees.of(-17.5), Degrees.of(0)));

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
        1.0, // Camera 1
        1.0, // Camera 2
      };
}
