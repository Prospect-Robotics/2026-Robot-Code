package com.team2813.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** Util class for getting information */
public class HubStatusUtil {
  private HubStatusUtil() {}

  /**
   * Code taken from <a
   * href=https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html#c-java-python>WPILib</a>
   *
   * @return <code>true</code> if our alliance's hub is active, <code>false</code> otherwise.
   */
  public static boolean isHubActive() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTimeInSeconds = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as it's likely early in
    // teleop.
    if (gameData.isEmpty()) {
      return true;
    }

    boolean redActiveFirst = true;
    switch (gameData.charAt(0)) {
      case 'R' -> redActiveFirst = false;
      case 'B' -> redActiveFirst = true;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> redActiveFirst;
          case Blue -> !redActiveFirst;
        };

    if (matchTimeInSeconds > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTimeInSeconds > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTimeInSeconds > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTimeInSeconds > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTimeInSeconds > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
  public static int timeLeftInCurrentPhase() {
    int matchTimeInSeconds =(int) DriverStation.getMatchTime();
    if(matchTimeInSeconds>130) {
      //transition
      return matchTimeInSeconds-130;
    }
    else if(matchTimeInSeconds>105) {
      //shift 1
      return matchTimeInSeconds-105;
    }
    else if(matchTimeInSeconds>80) {
      //shift 2
      return matchTimeInSeconds-80;
    }
    else if(matchTimeInSeconds>55) {
      //shift 3
      return matchTimeInSeconds-55;
    }
    else if(matchTimeInSeconds>30) {
      //shift 4
      return matchTimeInSeconds-30;
    }
    else if(matchTimeInSeconds>0) {
      //endgame
      return matchTimeInSeconds;
    }
    else return 0;
  }
}
