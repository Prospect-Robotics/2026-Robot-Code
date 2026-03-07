package com.team2813.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

public class HubPositionUtilTest {

    @Test
    public void doesDistanceToBlueHubCalculateRight() {
        Pose2d testPosition = new Pose2d(); // Rotation and Translation of 0.

        Distance distanceFromHub = HubPositionUtil.getBotToHubDistance(testPosition, Optional.of(DriverStation.Alliance.Blue));

        // The distance from bot to hub is sqrt(4^2+4.48^2), calculated by java it is this number.
        Assertions.assertEquals(6.080822312812635, distanceFromHub.in(Meters));
    }

    @Test
    public void doesDistanceToRedHubCalculateRight() {
        Pose2d testPosition = new Pose2d(); // Rotation and Translation of 0.

        Distance distanceFromHub = HubPositionUtil.getBotToHubDistance(testPosition, Optional.of(DriverStation.Alliance.Red));

        // The distance from bot to hub is sqrt(4^2+11.812^2), calculated by java it is this number.
        Assertions.assertEquals(12.47089988733772, distanceFromHub.in(Meters));
    }

    @Test
    public void doesAngleToBlueHubCalculateRight() {
        Pose2d testPosition = new Pose2d(); // Rotation and Translation of 0.

        Rotation2d angleFromHub = HubPositionUtil.getBotToHubAngle(testPosition, Optional.of(DriverStation.Alliance.Blue));

        Assertions.assertEquals(0.7179017820664226, angleFromHub.getRadians());
    }

    @Test
    public void doesAngleToRedHubCalculateRight() {
        Pose2d testPosition = new Pose2d();

        Rotation2d angleFromHub = HubPositionUtil.getBotToHubAngle(testPosition, Optional.of(DriverStation.Alliance.Red));

        Assertions.assertEquals(0.3265177360538555, angleFromHub.getRadians());
    }
}
