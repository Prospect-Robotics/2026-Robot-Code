package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Class that holds control logic and public interface for the elevator. */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged replayedInputs = new ClimbIOInputsAutoLogged();

  private InnerClimbHeight currentInnerClimbSetpoint = InnerClimbHeight.DOWN;
  private OuterClimbHeight currentOuterClimbSetpoint = OuterClimbHeight.DOWN;

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
    Logger.processInputs("Climb", replayedInputs);
    Logger.recordOutput(
        "Climb/Carriage Setpoint (inches)",
        currentInnerClimbSetpoint.getInnerPosition().in(Inches));
    Logger.recordOutput(
        "Climb/Motor Setpoint (rotations)",
        currentInnerClimbSetpoint.getInnerPositionAngle().in(Rotations));

    Logger.processInputs("Climb", replayedInputs);
    Logger.recordOutput(
        "Climb/Carriage Setpoint (inches)",
        currentOuterClimbSetpoint.getOuterPosition().in(Inches));
    Logger.recordOutput(
        "Climb/Motor Setpoint (rotations)",
        currentOuterClimbSetpoint.getOuterPositionAngle().in(Rotations));
  }

  @Override
  public void simulationPeriodic() {
    defaultSimulationVisualizerInstance.updateInnerClimbHeight(
        Inches.of(replayedInputs.innerCarriagePositionInches));
    defaultSimulationVisualizerInstance.updateOuterClimbHeight(
        Inches.of(replayedInputs.outerCarriagePositionInches));
  }

  public void stopInnerClimb() {
    io.stopInnerMotor();
  }

  public void stopOuterClimb() {
    io.stopOuterMotor();
  }

  public void setInnerClimbPosition(InnerClimbHeight heightSetpoint) {
    currentInnerClimbSetpoint = heightSetpoint;
    io.setInnerMotorSetpoint(heightSetpoint.getInnerPositionAngle());
  }

  public void setOuterClimbPosition(OuterClimbHeight heightSetpoint) {
    currentOuterClimbSetpoint = heightSetpoint;
    io.setOuterMotorSetpoint(heightSetpoint.getOuterPositionAngle());
  }

  public Command setInnerClimbPositionCommand(InnerClimbHeight height) {
    return new InstantCommand(() -> setInnerClimbPosition(height));
  }

  public Command setOuterClimbPositionCommand(OuterClimbHeight height) {
    return new InstantCommand(() -> setOuterClimbPosition(height));
  }

  public enum InnerClimbHeight {
    UP(Inches.of(9.75)),
    MIDDLE(Inches.of(4.875)),
    DOWN(Inches.of(0.0));

    public final Distance position;

    InnerClimbHeight(Distance position) {
      this.position = position;
    }

    public Distance getInnerPosition() {
      return position;
    }

    public Angle getInnerPositionAngle() {
      return Rotations.of(
          (getInnerPosition().in(Inches) / 2)
              / ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION);
    }
  }

  public enum OuterClimbHeight {
    UP(Inches.of(11)),
    MIDDLE(Inches.of(5.5)),
    DOWN(Inches.of(0.0));

    public final Distance position;

    OuterClimbHeight(Distance position) {
      this.position = position;
    }

    public Distance getOuterPosition() {
      return position;
    }

    public Angle getOuterPositionAngle() {
      return Rotations.of(
          (getOuterPosition().in(Inches) / 2)
              / ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION);
    }
  }
}
