package com.team2813.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** Utility class for calculating robot distance from / rotation to the Hub. */
public class HubPositionUtil {

  private HubPositionUtil() {}

  public static final Pose2d BLUE_HUB_POSITION = new Pose2d(4.580, 4.000, Rotation2d.kZero);
  public static final Pose2d RED_HUB_POSITION = new Pose2d(11.812, 4.000, Rotation2d.kZero);

  /**
   * Calculates the angle to the hub (based on the current alliance). Defaults to blue hub if no
   * alliance present.
   *
   * @param robotPosition The current robot position in a {@link Pose2d}.
   * @param currentAlliance The current alliance, use with {@link DriverStation#getAlliance()}
   * @return The {@link Rotation2d} to the hub.
   */
  public static Rotation2d getBotToHubAngle(
      Pose2d robotPosition, Optional<DriverStation.Alliance> currentAlliance) {
    Pose2d hub = getCurrentHub(currentAlliance);
    return hub.getTranslation().minus(robotPosition.getTranslation()).getAngle();
  }

  /**
   * Calculates the distance to the hub (based on the current alliance). Defaults to blue hub if no
   * alliance present.
   *
   * @param robotPosition The current robot position in a {@link Pose2d}.
   * @param currentAlliance The current alliance, use with {@link DriverStation#getAlliance()}
   * @return The distance from the center of the current Hub.
   */
  public static Distance getBotToHubDistance(
      Pose2d robotPosition, Optional<DriverStation.Alliance> currentAlliance) {
    Pose2d hub = getCurrentHub(currentAlliance);
    Translation2d robotToHubTranslation =
        hub.getTranslation().minus(robotPosition.getTranslation());

    robotToHubTranslation.getNorm();

    // Translation2d keeps x and y in meters.
    return Meters.of(robotToHubTranslation.getNorm());
  }

  /**
   * @param currentAlliance The current Alliance as an {@link Optional}, intended for use with
   *     {@link DriverStation#getAlliance()}
   * @return The {@link Pose2d} of the hub for the current alliance. Defaults to blue if there is no
   *     alliance.
   */
  private static Pose2d getCurrentHub(Optional<DriverStation.Alliance> currentAlliance) {
    return currentAlliance.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
        ? RED_HUB_POSITION
        : BLUE_HUB_POSITION;
  }
}
