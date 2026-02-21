package com.team2813;

import static com.google.common.truth.Truth.assertThat;

import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junitpioneer.jupiter.ClearEnvironmentVariable;
import org.junitpioneer.jupiter.SetEnvironmentVariable;

public class RobotTest {
  private static final String LOG_REPLAY_ENABLE_ENV_NAME = "FRC_ADVANTAGEKIT_LOG_REPLAY_ENABLE";

  @Nested
  public class GetCurrentModeFromEnvTest {

    @Test
    @ClearEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME)
    public void unsetEnvInSimulation() {
      setRuntimeType(RuntimeType.kSimulation);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.SIM);
    }

    @Test
    @ClearEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME)
    public void unsetEnvWithRealRobot() {
      setRuntimeType(RuntimeType.kRoboRIO);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.REAL);
    }

    @Test
    @SetEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME, value = "false")
    public void falseEnvInSimulation() {
      setRuntimeType(RuntimeType.kSimulation);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.SIM);
    }

    @Test
    @SetEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME, value = "false")
    public void falseEnvWithRealRobot() {
      setRuntimeType(RuntimeType.kRoboRIO);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.REAL);
    }

    @Test
    @SetEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME, value = "true")
    public void trueEnvInSimulation() {
      setRuntimeType(RuntimeType.kSimulation);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.REPLAY);
    }

    @Test
    @SetEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME, value = "True")
    public void capitalTrueEnvInSimulation() {
      setRuntimeType(RuntimeType.kSimulation);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.REPLAY);
    }

    @Test
    @SetEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME, value = "true")
    public void trueEnvWithRealRobot() {
      setRuntimeType(RuntimeType.kRoboRIO);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.REAL);
    }

    @Test
    @SetEnvironmentVariable(key = LOG_REPLAY_ENABLE_ENV_NAME, value = "random_invalid_env_value")
    public void randomEnvInSimulation() {
      setRuntimeType(RuntimeType.kSimulation);
      assertThat(Robot.getCurrentModeFromEnv()).isEqualTo(Mode.SIM);
    }
  }

  private static void setRuntimeType(RuntimeType runtimeType) {
    SimHooks.setHALRuntimeType(runtimeType.value);
  }
}
