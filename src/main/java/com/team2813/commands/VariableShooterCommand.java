package com.team2813.commands;

import static edu.wpi.first.units.Units.*;

import com.team2813.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import java.util.function.Supplier;

public class VariableShooterCommand {
  private static final AngularVelocity MIN_SPEED =
      RotationsPerSecond.of(60); // 0.5 meters. Hub shot speed.
  private static final AngularVelocity MAX_SPEED =
      RotationsPerSecond.of(100); // ~2.5 meters from hub speed.

  public static final Distance MIN_DIST = Meters.of(1.5);
  public static final Distance MAX_DIST = Meters.of(3.1);

  private static void shootBasedOnDistance(Shooter shooter, Supplier<Distance> distanceSupplier) {
    Distance distance = distanceSupplier.get();

    double speedSlope = (distance.minus(MIN_DIST)).div(MAX_DIST.minus(MIN_DIST)).magnitude();
    speedSlope = MathUtil.clamp(speedSlope, 0, 1);
    AngularVelocity velocity = MIN_SPEED.plus((MAX_SPEED.minus(MIN_SPEED)).times(speedSlope));

    shooter.setShooterMotorVelocity(velocity);
  }

  public static Command shootBasedOnDistanceCommand(
      Shooter shooter, Supplier<Distance> distanceSupplier) {
    return new RepeatCommand(
            new InstantCommand(() -> shootBasedOnDistance(shooter, distanceSupplier)))
        .finallyDo(shooter::stop).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
}
