// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.team2813.subsystems.vision;

import static com.team2813.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      VisionSystemSim visionSystemSim) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (this.visionSim == null) {
      this.visionSim = new VisionSystemSim("main");
      this.visionSim.addAprilTags(APRIL_TAG_LAYOUT);
    } else {
      this.visionSim = visionSystemSim;
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, APRIL_TAG_LAYOUT);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
