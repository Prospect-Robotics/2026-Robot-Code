package com.team2813;

import static com.google.common.truth.Truth.assertThat;

import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junitpioneer.jupiter.ClearEnvironmentVariable;
import org.junitpioneer.jupiter.SetEnvironmentVariable;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class GetSimModeTest {
  // The first test must run with "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE" unset, so that it doesn't
  // stay in a different mode
  @Test
  @ClearEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE")
  @Order(1)
  public void simModeIsCurrentMode() {
    assertThat(Constants.currentMode).isEqualTo(Constants.getSimMode());
  }

  @Test
  @ClearEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE")
  @Order(1)
  public void unsetEnv() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.SIM);
  }

  @Test
  @SetEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE", value = "false")
  @Order(2)
  public void falseEnv() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.SIM);
  }

  @Test
  @SetEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE", value = "true")
  @Order(2)
  public void trueEnv() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.REPLAY);
  }

  @Test
  @SetEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE", value = "True")
  @Order(2)
  public void capitalTrueEnv() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.REPLAY);
  }

  @Test
  @SetEnvironmentVariable(
      key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE",
      value = "random_invalid_env_value")
  @Order(2)
  public void randomEnv() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.SIM);
  }
}
