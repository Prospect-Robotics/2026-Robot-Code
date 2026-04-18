package com.team2813.commands;

import static edu.wpi.first.units.Units.*;

import com.team2813.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.Supplier;

/** Helper class allowing for automatically adjusting shooter speed based on hub distance. */
public class VariableShooterCommand {
  // TODO: Tweak the speeds and distance after pinnacles if we have time.
  private static final AngularVelocity MIN_SPEED =
      RotationsPerSecond.of(36); // 2 meters. Hub shot speed.
  private static final AngularVelocity MAX_SPEED =
      RotationsPerSecond.of(60); // 5 meters from hub speed.

  // Hub to square edge is 1.3 based on the HubStatus/Utils calculation meters.
  // Get these values off of the /AdvantageKit/RealOutputs/HubStatus/Distance To Our Hub (Meters)
  public static final Distance MIN_DIST = Meters.of(2.0);
  public static final Distance MAX_DIST = Meters.of(5.0);

  /**
   * Calculates the speed to shoot at if between the 1.5/3.1 MIN/MAX distance.
   *
   * @param shooter Instance of the shooter class to apply the calculated speed to.
   * @param distanceSupplier A supplier of the distance from the hub
   * @see com.team2813.util.HubPositionUtil For getting distance from hub.
   */
  private static void shootBasedOnDistance(Shooter shooter, Supplier<Distance> distanceSupplier) {
    Distance distance = distanceSupplier.get();

    double speedSlope = (distance.minus(MIN_DIST)).div(MAX_DIST.minus(MIN_DIST)).magnitude();
    speedSlope = MathUtil.clamp(speedSlope, 0, 1);
    AngularVelocity velocity = MIN_SPEED.plus((MAX_SPEED.minus(MIN_SPEED)).times(speedSlope));

    shooter.setShooterMotorVelocity(velocity);
  }

  public static Command shootBasedOnDistanceCommand(
      Shooter shooter, Supplier<Distance> distanceSupplier) {
    return new RunCommand(() -> shootBasedOnDistance(shooter, distanceSupplier), shooter)
        .finallyDo(shooter::stop)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }
}
