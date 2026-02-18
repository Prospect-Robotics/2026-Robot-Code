package com.team2813;

import static com.google.common.truth.Truth.assertThat;

import org.junit.jupiter.api.Test;
import org.junitpioneer.jupiter.ClearEnvironmentVariable;
import org.junitpioneer.jupiter.SetEnvironmentVariable;

public class GetSimModeTest {
  @Test
  @ClearEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE")
  public void noSetEnvTest() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.SIM);
  }

  @Test
  @SetEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE", value = "false")
  public void envSetToFalseTest() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.SIM);
  }

  @Test
  @SetEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE", value = "true")
  public void envSetToLowercaseTrueTest() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.REPLAY);
  }

  @Test
  @SetEnvironmentVariable(key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE", value = "True")
  public void envSetToCapitalTrueTest() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.REPLAY);
  }

  @Test
  @SetEnvironmentVariable(
      key = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE",
      value =
          "V2h5IGRpZCB5b3UgZ28gdGhyb3VnaCBhbGwgb2YgdGhlIGVmZm9ydCB0byBkZWNv\n"
              + "ZGUgdGhpcz8/Pwo=\n"
              + "=71AM\n")
  public void randomEnvTest() {
    assertThat(Constants.getSimMode()).isEqualTo(Constants.Mode.SIM);
  }
}
