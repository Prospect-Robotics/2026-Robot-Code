/*
Copyright 2025-2026 Prospect Robotics SWENext Club

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
package com.team2813;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.extension.*;

/**
 * JUnit Jupiter extension for testing code that depends on WPILib.
 *
 * <p>Also provides a {@link CommandTester} for tests.
 *
 * <p>Example use:
 *
 * <pre>{@code
 * @ExtendWith(WPILibExtension.class)
 * public final class FlightSubsystemTest {
 *
 *   @Test
 *   public void initiallyNotInAir() {
 *     var flight = new FlightSubsystem();
 *
 *     assertThat(flight.inAir()).isFalse();
 *   }
 *
 *   @Test
 *   public void takesFlight(CommandTester commandTester) {
 *     var flight = new FlightSubsystem();
 *     Command takeOff = flight.createTakeOffCommandCommand();
 *
 *     commandTester.runUntilComplete(takeOff);
 *
 *     assertThat(flight.inAir()).isTrue();
 *   }
 * }
 * }</pre>
 *
 * @since 2.0.0
 */
// This is taken from
// [lib2813](https://github.com/Prospect-Robotics/lib2813/blob/main/testing/src/main/java/com/team2813/lib2813/testing/junit/jupiter/WPILibExtension.java).
// When lib2813 is on maven central, this file can be deleted in favor of adding a dependency on
// lib2813.
public final class WPILibExtension
    implements Extension,
        AfterAllCallback,
        AfterEachCallback,
        BeforeAllCallback,
        ParameterResolver {
  private static final double NANOS_PER_SECOND = 1_000_000_000d;

  @Override
  public void beforeAll(ExtensionContext context) {
    // See https://www.chiefdelphi.com/t/driverstation-getalliance-in-gradle-test/
    if (!HAL.initialize(500, 0)) {
      throw new IllegalStateException("Could not initialize Hardware Abstraction Layer");
    }
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.setHALRuntimeType(RuntimeType.kSimulation.value);

    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    commandScheduler.enable();
    commandScheduler.cancelAll();
    commandScheduler.unregisterAllSubsystems();
  }

  @Override
  public void afterEach(ExtensionContext context) {
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    commandScheduler.cancelAll();
    commandScheduler.unregisterAllSubsystems();
  }

  @Override
  public void afterAll(ExtensionContext context) {
    CommandScheduler commandScheduler = CommandScheduler.getInstance();
    commandScheduler.cancelAll();
    commandScheduler.unregisterAllSubsystems();
    commandScheduler.disable();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Override
  public boolean supportsParameter(
      ParameterContext parameterContext, ExtensionContext extensionContext)
      throws ParameterResolutionException {
    return CommandTester.class.equals(parameterContext.getParameter().getType());
  }

  @Override
  public CommandTester resolveParameter(
      ParameterContext parameterContext, ExtensionContext extensionContext) {
    CommandScheduler scheduler = CommandScheduler.getInstance();

    return command -> {
      SimHooks.pauseTiming();
      try {
        scheduler.schedule(command);
        do {
          scheduler.run();
          SimHooks.stepTiming(TimedRobot.kDefaultPeriod);
        } while (scheduler.isScheduled(command));
      } finally {
        SimHooks.resumeTiming();
      }
    };
  }
}
