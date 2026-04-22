package com.team2813;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class RobotTeleopRumbleTest {
  @ParameterizedTest
  @CsvSource({
    // Active hub shift: rumble only when 4 < time <= 5.
    "true,5.1,false",
    "true,5.0,true",
    "true,4.5,true",
    "true,4.0,false",
    "true,0.0,false",
    // Inactive hub shift: rumble only when 0 < time <= 2.
    "false,2.1,false",
    "false,2.0,true",
    "false,1.0,true",
    "false,0.0,false",
    "false,-0.1,false"
  })
  void shouldRumbleMatchesTeleopPeriodicLogic(boolean hubActive, double timeLeft, boolean expected) {
    if (expected) {
      assertTrue(Robot.shouldRumble(hubActive, timeLeft));
    } else {
      assertFalse(Robot.shouldRumble(hubActive, timeLeft));
    }
  }
}

