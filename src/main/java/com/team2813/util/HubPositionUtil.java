package com.team2813.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Utility class for calculating robot distance from / rotation to the Hub. */
public class HubPositionUtil {

  private HubPositionUtil() {}

  public static final Pose2d BLUE_HUB_POSITION = new Pose2d(4.580, 4.000, Rotation2d.kZero);
  public static final Pose2d RED_HUB_POSITION = new Pose2d(11.812, 4.000, Rotation2d.kZero);

  public static Rotation2d getBotToHubAngle(Pose2d robotPosition, DriverStation.Alliance alliance) {
    Pose2d hub;

    if (alliance == DriverStation.Alliance.Red) {
      hub = RED_HUB_POSITION;
    } else {
      hub = BLUE_HUB_POSITION;
    }

    return hub.getTranslation().minus(robotPosition.getTranslation()).getAngle();
  }
}
