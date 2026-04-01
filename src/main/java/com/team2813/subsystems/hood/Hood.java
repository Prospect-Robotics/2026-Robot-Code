package com.team2813.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase implements AutoCloseable {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged replayedInputs = new HoodIOInputsAutoLogged();
  private boolean atPosition = true;

  public Hood(HoodIO io) {
    this.io = Objects.requireNonNull(io, "io");
  }

  public Command gotoAngleCommand(Angle angle) {
    return new StartEndCommand(() -> gotoAngle(angle), () -> {}, this).until(this::atPosition);
  }

  public Command gotoAngleCommand(Supplier<Angle> angleSupplier) {
    return new DeferredCommand(() -> gotoAngleCommand(angleSupplier.get()), Set.of(this));
  }

  private void gotoAngle(Angle angle) {
    io.setSetpoint(transformAngle(angle));
  }

  public boolean atPosition() {
    return atPosition;
  }

  /**
   * Transform an angle between the angle to shoot and the angle of the shooter. This operation is
   * symmetrical, so inputting an angle from either reference point will give the angle of the
   * other. The behavior of this function is undefined if the angle provided is not an angle that
   * can be reached physically.
   *
   * @param angle The angle in either reference point
   * @return The angle in the other reference point
   */
  private Angle transformAngle(Angle angle) {
    return Radians.of(Math.PI / 2 - 0.284256).minus(angle);
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    double error = replayedInputs.motorAngle.minus(replayedInputs.motorSetpoint).abs(Radians);

    atPosition = error < Math.PI;
    Logger.recordOutput("Hood/atPosition", atPosition);
    Logger.recordOutput("Hood/shootAngle", transformAngle(replayedInputs.motorAngle));
    Logger.processInputs("Hood", replayedInputs);
  }

  @Override
  public void close() throws Exception {
    io.close();
  }
}
