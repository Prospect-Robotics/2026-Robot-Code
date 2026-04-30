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
import edu.wpi.first.wpilibj.Preferences;
import org.littletonrobotics.junction.Logger;

public class VisionConstants {

  public static final String VISION_ENABLED_NT = "Vision/VisionEnabled";

  static {
    Preferences.initBoolean(VISION_ENABLED_NT, true);
  }

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  // See documentation/images/camera_position.png for a diagram of the camera position layout on the
  // robot.
  public static final String RED_BACK_LEFT_COLOR_CAMERA_NAME = "red_back_left_color";
  public static final String GREEN_BACK_RIGHT_COLOR_CAMERA_NAME = "green_back_right_color";
  public static final String BLUE_FRONT_MONO_CAMERA_NAME = "blue_front_monochrome";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d RED_BACK_LEFT_CAM_FROM_ROBOT =
      new Transform3d(
          new Translation3d(Inches.of(-7.802), Inches.of(-13.188), Inches.of(7.125)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(-55)));

  public static final Transform3d GREEN_BACK_RIGHT_CAM_FROM_ROBOT =
      new Transform3d(
          new Translation3d(Inches.of(-7.802), Inches.of(13.188), Inches.of(7.125)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(55)));

  public static final Transform3d BLUE_FRONT_CAM_FROM_ROBOT =
      new Transform3d(
          new Translation3d(Inches.of(-13.263), Centimeters.of(0), Inches.of(14.028)),
          new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(180)));

//  static {
//    Logger.recordOutput("VisTest/Red", RED_BACK_LEFT_CAM_FROM_ROBOT);
//    Logger.recordOutput("VisTest/Green", GREEN_BACK_RIGHT_CAM_FROM_ROBOT);
//    Logger.recordOutput("VisTest/Blue", BLUE_FRONT_CAM_FROM_ROBOT);
//  }

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

  /**
   * @return true if vision is enabled, or false if not or if the Preferences networktable doesn't
   *     exist
   */
  public static boolean isVisionEnabled() {
    return Preferences.getBoolean(VISION_ENABLED_NT, false);
  }

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
