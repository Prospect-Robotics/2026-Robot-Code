package com.team2813.util;

import static com.google.common.truth.Truth.assertThat;
import static com.team2813.lib2813.testing.truth.Rotation2dSubject.assertThat;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class HubPositionUtilTest {

  @ParameterizedTest(name = "{0}, {1}")
  @MethodSource("allData")
  public void angleCalculation(DriverStation.Alliance alliance, TestData data) {
    Rotation2d angleFromHub =
        HubPositionUtil.getBotToHubAngle(data.testPosition, Optional.of(alliance));
    assertThat(angleFromHub).isWithin(1e-5).of(data.getAngle(alliance));
  }

  @ParameterizedTest(name = "{0}, {1}")
  @MethodSource("allData")
  public void distanceCalculation(DriverStation.Alliance alliance, TestData data) {
    Distance distanceToHub =
        HubPositionUtil.getBotToHubDistance(data.testPosition, Optional.of(alliance));
    assertThat(distanceToHub.in(Meters)).isWithin(1e-5).of(data.getDistance(alliance));
  }

  public record TestData(
      Pose2d testPosition,
      double expectedRedAngle,
      double expectedRedDistance,
      double expectedBlueAngle,
      double expectedBlueDistance) {
    @Override
    public String toString() {
      return testPosition.toString();
    }

    public Rotation2d getAngle(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Blue) {
        return new Rotation2d(expectedBlueAngle);
      } else {
        return new Rotation2d(expectedRedAngle);
      }
    }

    public double getDistance(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Blue) {
        return expectedBlueDistance;
      } else {
        return expectedRedDistance;
      }
    }
  }

  static Stream<Arguments> allData() {
    TestData[] data = {
      new TestData(
          Pose2d.kZero,
          0.3265177360538555,
          12.47089988733772,
          0.7179017820664226,
          6.080822312812635),
      new TestData(
          new Pose2d(15, 5, Rotation2d.kZero),
          -2.8376365100925645,
          3.3411590803192835,
          -3.0459163753242049,
          10.4678746648973592),
      new TestData(
          new Pose2d(5, 2, Rotation2d.kZero),
          0.2855745093824902,
          7.0995312521320730,
          1.7777885210147176,
          2.0436242316042350)
    };
    return Stream.of(DriverStation.Alliance.values())
        .flatMap(
            (alliance) -> {
              return Stream.of(data).map((testData) -> Arguments.of(alliance, testData));
            });
  }
}
