# IntakeExtension Test Fix Report

**Date:** February 23, 2026
**Author:** GitHub Copilot
**Affected Files:**
- `IntakeExtensionTest.java`
- `IntakeExtensionIOSim.java`
- `IntakeExtension.java`

---

## Problem Summary

The `IntakeExtensionTest` tests were failing **inconsistently** - sometimes one test would pass, sometimes both would fail, and sometimes both would pass. The user suspected static state leakage between tests.

### Failing Tests
- `testIntakeExtensionIsAtPositionExtend()`
- `testIntakeExtensionIsAtPositionRetract()`

### Observed Behavior
```
IntakeExtensionTest > testIntakeExtensionIsAtPositionExtend() FAILED
    java.lang.AssertionError: expected:<false> but was:<true>

IntakeExtensionTest > testIntakeExtensionIsAtPositionRetract() FAILED
    java.lang.AssertionError: expected:<false> but was:<true>
```

---

## Root Cause Analysis

### Issue 1: Static State Leakage Between Tests (Minor)

The original test class used `@BeforeAll` for HAL initialization:

```java
@BeforeAll
public static void init() {
    HAL.initialize(500, 1);
    // ...
}
```

This meant the HAL and CTRE Phoenix's static device registry were only initialized once for the entire test class. When multiple tests created `TalonFX` instances with the same CAN ID, the state from previous tests could leak into subsequent tests.

### Issue 2: TalonFX Simulation Doesn't Run Internal PID (Major - Root Cause)

**This was the primary issue.** After isolating the tests and adding debug output, we discovered:

```
Position (rotations): 0.0
Setpoint (rotations): 10.504226244065094
Error (rotations): 10.504226244065094
isExtenderAtPosition: false
```

The **motor position was stuck at 0.0** despite calling `extend()` and running 50 simulation iterations.

#### Why This Happened

The original `IntakeExtensionIOSim` relied on `TalonFXSimState.getMotorVoltage()` to get the PID-computed voltage:

```java
// Original code - DOES NOT WORK
extenderSim.setInput(MOTOR_DIRECTION * extenderMotorSimState.getMotorVoltage());
```

**However, CTRE Phoenix 6's `TalonFXSimState` does NOT run the motor's internal PID controller in simulation.** The `getMotorVoltage()` method always returned `0.0`, meaning no voltage was ever applied to the `ElevatorSim`, and the simulated mechanism never moved.

This is a key difference from real hardware where the TalonFX handles PID internally. In simulation, you must implement the control loop yourself.

---

## Solution

### Fix 1: Per-Test HAL Initialization

Changed from `@BeforeAll` to `@BeforeEach` / `@AfterEach` with proper cleanup:

```java
@BeforeEach
public void setUp() {
    HAL.initialize(500, 1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.setHALRuntimeType(RuntimeType.kSimulation.value);

    intakeExtension = new IntakeExtension(new IntakeExtensionIOSim());
}

@AfterEach
public void tearDown() {
    if (intakeExtension != null) {
        intakeExtension.close();
        intakeExtension = null;
    }
    HAL.shutdown();  // Clears all static device state
}
```

### Fix 2: Implement PID Control in Simulation

Added a WPILib `PIDController` to simulate the motor's position control:

```java
// Initialize PID controller with same gains as motor config
var slot0 = IntakeExtensionConstants.EXTENDER_MOTOR_CONFIG.Slot0;
simPidController = new PIDController(slot0.kP, slot0.kI, slot0.kD);
```

Updated `updateState()` to use the WPILib PID controller:

```java
// Calculate position error and PID output
double currentPositionRotations = extenderSim.getPositionMeters()
    * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS;
double setpointRotations = extensionSetpoint.in(Rotations);

double pidOutput = simPidController.calculate(currentPositionRotations, setpointRotations);

// Add feedforward and clamp to supply voltage
double feedforward = Math.signum(pidOutput) * slot0.kS;
double motorVoltage = MathUtil.clamp(pidOutput + feedforward, -12.0, 12.0);

extenderSim.setInput(motorVoltage);
```

### Fix 3: Report Position Directly from ElevatorSim

To avoid TalonFX inversion issues, position is now reported directly from the physics simulation:

```java
inputs.extenderMotorPosition = Rotations.of(
    extenderSim.getPositionMeters() * IntakeExtensionConstants.DISTANCE_METERS_TO_MOTOR_ROTATIONS);
```

---

## Verification

After applying the fixes, all tests pass consistently:

```
IntakeExtensionTest > testIntakeExtensionExtend() PASSED
IntakeExtensionTest > testIntakeExtensionRetract() PASSED
IntakeExtensionTest > testIntakeExtensionIsAtPositionExtend() PASSED
IntakeExtensionTest > testIntakeExtensionIsAtPositionRetract() PASSED

BUILD SUCCESSFUL
```

Debug output confirms the simulation is working correctly:
```
Position (rotations): 10.265493829427252
Setpoint (rotations): 10.504226244065094
Error (rotations): 0.2387324146378429
isExtenderAtPosition: true
```

The small error (0.24 rotations) is expected because the `ElevatorSim` is physically clamped at `EXTENDED_POSITION` (10.75 inches), while the setpoint `OUT` includes an additional `ANTI_STALL_DISTANCE` (0.25 inches) beyond the physical limit. This error is within the 0.4 rotation tolerance defined in `IntakeExtension.periodic()`.

---

## Key Learnings

1. **CTRE Phoenix 6 Simulation Limitation:** The `TalonFXSimState` does not execute the motor's internal PID controller. You must implement your own control loop for simulation.

2. **HAL State Persistence:** The WPILib HAL and CTRE device registries maintain static state that persists across test methods unless explicitly cleared with `HAL.shutdown()`.

3. **Test Isolation:** Always use `@BeforeEach` / `@AfterEach` for proper test isolation when dealing with hardware simulation.

4. **Consistent Patterns:** Other simulation files in this codebase (e.g., `ModuleIOSim.java`) already use WPILib `PIDController` for simulation control loops - this is the established pattern.

---

## Files Changed

| File | Changes |
|------|---------|
| `IntakeExtensionTest.java` | Changed to `@BeforeEach`/`@AfterEach`, added `HAL.shutdown()`, simplified test methods |
| `IntakeExtensionIOSim.java` | Added WPILib `PIDController` for simulation, report position from `ElevatorSim` |
| `IntakeExtension.java` | Added `getPosition()` method for testing/debugging |
