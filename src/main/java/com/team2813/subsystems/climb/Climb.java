package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Class that holds control logic and public interface for the elevator. */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged replayedInputs = new ClimbIOInputsAutoLogged();

  private ClimbHeight currentClimbSetpoint = ClimbHeight.DOWN;

  private final SimulationVisualizer defaultSimulationVisualizerInstance =
      SimulationVisualizer.getInstance();

  /**
   * @param io The hardware implementation for the climb, either sim or real.
   */
  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);
    // `processInputs` must be called every periodic after updating hardware state.
    // In `REPLAY` mode, `updateState` does nothing, and the `replayedInputs` are populated from the
    // replayed logs
    // instead.
    Logger.processInputs(String.format("Climb/%s", io.climbConstants.climbName()), replayedInputs);
    Logger.recordOutput(
        String.format("Climb/%s/Carriage Setpoint (inches)", io.climbConstants.climbName()),
        currentClimbSetpoint.getPosition().in(Inches));
    Logger.recordOutput(
        String.format("Climb/%s/Motor Setpoint (rotations)", io.climbConstants.climbName()),
        currentClimbSetpoint.getPositionAngle().in(Rotations));
  }

  @Override
  public void simulationPeriodic() {
    defaultSimulationVisualizerInstance.updateOuterClimbHeight(
        Inches.of(replayedInputs.carriagePositionInches));
  }

  public void stopClimb() {
    io.stopMotor();
  }

  public void setClimbPosition(ClimbHeight heightSetpoint) {
    currentClimbSetpoint = heightSetpoint;
    io.setMotorSetpoint(heightSetpoint.getPositionAngle());
  }

  public void setMotorVoltage(Voltage motorVoltage) {
    io.setMotorVoltage(motorVoltage);
  }

  public Command setClimbPositionCommand(ClimbHeight height) {
    return new InstantCommand(() -> setClimbPosition(height));
  }

  // TODO: Move these into a different class as we split climb into two instances.
  //  public Command manuelDownInnerClimb() {
  //    return new InstantCommand(() -> setInnerMotorVoltage(upMotorVoltage.div(-1)));
  //  }
  //
  //  public Command manuelUpInnerClimb() {
  //    return new InstantCommand(() -> setInnerMotorVoltage(upMotorVoltage));
  //  }
  //
  //  public Command postAutoClimb() {
  //    return setInnerClimbPositionCommand(InnerClimbHeight.POSTAUTO);
  //  }
  //
  //  public Command deployClimb() {
  //    return new SequentialCommandGroup(
  //        setOuterClimbPositionCommand(OuterClimbHeight.UP),
  //        setInnerClimbPositionCommand(InnerClimbHeight.UP));
  //  }
  //
  //  public Command l1Sequence() {
  //    return setInnerClimbPositionCommand(InnerClimbHeight.DOWN);
  //  }
  //
  //  public Command l2Sequence() {
  //    return new SequentialCommandGroup(
  //        l1Sequence(),
  //        new WaitCommand(1),
  //        setOuterClimbPositionCommand(OuterClimbHeight.DOWN),
  //        new WaitCommand(1),
  //        setInnerClimbPositionCommand(InnerClimbHeight.UP),
  //        new WaitCommand(1),
  //        setInnerClimbPositionCommand(InnerClimbHeight.DOWN),
  //        new WaitCommand(1),
  //        setOuterClimbPositionCommand(OuterClimbHeight.UP));
  //  }
  //
  //  public Command l3Sequence() {
  //    return new SequentialCommandGroup(
  //        l2Sequence(),
  //        new WaitCommand(1),
  //        setOuterClimbPositionCommand(OuterClimbHeight.DOWN),
  //        new WaitCommand(1),
  //        setInnerClimbPositionCommand(InnerClimbHeight.UP),
  //        new WaitCommand(1),
  //        setInnerClimbPositionCommand(InnerClimbHeight.DOWN),
  //        new WaitCommand(1),
  //        setOuterClimbPositionCommand(OuterClimbHeight.UP));
  //  }
  //
  //  public enum InnerClimbHeight {
  //    // elliot said add 3 inches since its not a normal elevator beacuse a rope is spolling it,
  //    // except for down
  //    // Origional values UP(Inches.of(9.75)), MIDDLE(Inches.of(4.875)),
  //    UP(Inches.of(12.75)),
  //    // TODO figure post auto position
  //    POSTAUTO(Inches.of(4)),
  //    MIDDLE(Inches.of(7.875)),
  //    DOWN(Inches.of(0.0));
  //
  //    public final Distance position;
  //
  //    InnerClimbHeight(Distance position) {
  //      this.position = position;
  //    }
  //
  //    public Distance getInnerPosition() {
  //      return position;
  //    }
  //
  //    public Angle getInnerPositionAngle() {
  //      return Rotations.of(
  //          position.in(Inches)
  //              / ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Inches));
  //    }
  //  }
  //
  public enum ClimbHeight {
    // elliot said add 3 inches since its not a normal elevator beacuse a rope is spolling it,
    // except for down
    // Origional values UP(Inches.of(11)), MIDDLE(Inches.of(5.5)),
    UP(Inches.of(14)),
    MIDDLE(Inches.of(8.5)),
    DOWN(Inches.of(0.0));

    public final Distance position;

    ClimbHeight(Distance position) {
      this.position = position;
    }

    public Distance getPosition() {
      return position;
    }

    public Angle getPositionAngle() {
      return Rotations.of(
          position.in(Inches)
              / ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Inches));
    }
  }
}
