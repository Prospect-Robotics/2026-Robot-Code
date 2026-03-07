package com.team2813.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class HubPositionUtilTest {

  @Test
  public void doesDistanceToBlueHubCalculateRight() {
    Pose2d testPosition = Pose2d.kZero;

    Distance distanceFromHub =
        HubPositionUtil.getBotToHubDistance(testPosition, Optional.of(DriverStation.Alliance.Blue));

    // The distance from bot to hub is sqrt(4^2+4.48^2), calculated by java it is this number.
    Assertions.assertEquals(6.080822312812635, distanceFromHub.in(Meters), 1e-5);
  }

  @Test
  public void doesDistanceToRedHubCalculateRight() {
    Pose2d testPosition = Pose2d.kZero;

    Distance distanceFromHub =
        HubPositionUtil.getBotToHubDistance(testPosition, Optional.of(DriverStation.Alliance.Red));

    // The distance from bot to hub is sqrt(4^2+11.812^2), calculated by java it is this number.
    Assertions.assertEquals(12.47089988733772, distanceFromHub.in(Meters), 1e-5);
  }

  @Test
  public void doesAngleToBlueHubCalculateRight() {
    Pose2d testPosition = Pose2d.kZero;

    Rotation2d angleFromHub =
        HubPositionUtil.getBotToHubAngle(testPosition, Optional.of(DriverStation.Alliance.Blue));

    Assertions.assertEquals(0.7179017820664226, angleFromHub.getRadians(), 1e-5);
  }

  @Test
  public void doesAngleToRedHubCalculateRight() {
    Pose2d testPosition = Pose2d.kZero;

    Rotation2d angleFromHub =
        HubPositionUtil.getBotToHubAngle(testPosition, Optional.of(DriverStation.Alliance.Red));

    Assertions.assertEquals(0.3265177360538555, angleFromHub.getRadians(), 1e-5);
  }
}
