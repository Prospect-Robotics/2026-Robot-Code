// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = VisionConstants.APRIL_TAG_LAYOUT.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      processObservations(cameraIndex, robotPoses, robotPosesAccepted, robotPosesRejected);

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /**
   * Processes the observations from each camera, sending accepted observations to the consumer and
   * logging all observations.
   *
   * @param cameraIndex - the index of the camera to process observations for
   * @param robotPoses - the list of robot poses to add all observations from selected camera to
   * @param robotPosesAccepted - the list of robot poses to add accepted observations from selected
   *     camera to
   * @param robotPosesRejected - the list of robot poses to add rejected observations from selected
   *     camera to
   */
  private void processObservations(
      int cameraIndex,
      List<Pose3d> robotPoses,
      List<Pose3d> robotPosesAccepted,
      List<Pose3d> robotPosesRejected) {
    // Loop over pose observations
    for (var observation : inputs[cameraIndex].poseObservations) {
      // Check whether to reject pose
      boolean rejectPose = shouldRejectPose(observation);

      // Add pose to log
      robotPoses.add(observation.pose());
      if (rejectPose) {
        robotPosesRejected.add(observation.pose());
      } else {
        robotPosesAccepted.add(observation.pose());
      }

      // Skip if rejected
      if (rejectPose) {
        continue;
      }

      // Calculate standard deviations
      double stdDevFactor =
          Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
      double linearStdDev = VisionConstants.LINEAR_STD_DEV_BASELINE * stdDevFactor;
      double angularStdDev = VisionConstants.ANGULAR_STD_DEV_BASELINE * stdDevFactor;
      if (cameraIndex < VisionConstants.CAMERA_STD_DEV_FACTORS.length) {
        linearStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
        angularStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
      }

      // Send vision observation
      consumer.accept(
          observation.pose().toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }
  }

  /**
   * Determines whether to reject a vision pose observation based on various criteria such as
   * ambiguity, Z coordinate, and field boundaries.
   *
   * @param observation
   * @return - true if the pose observation should be rejected, false otherwise
   */
  private static boolean shouldRejectPose(VisionIO.PoseObservation observation) {
    return observation.tagCount() == 0 // Must have at least one tag
        || (observation.tagCount() == 1
            && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) // Cannot be high ambiguity
        || Math.abs(observation.pose().getZ())
            > VisionConstants.MAX_Z_ERROR // Must have realistic Z coordinate

        // Must be within the field boundaries
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > VisionConstants.APRIL_TAG_LAYOUT.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth();
  }
}
