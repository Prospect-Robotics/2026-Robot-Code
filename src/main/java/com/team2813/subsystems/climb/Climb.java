package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;

import com.team2813.util.SimulationVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;

/** Class that holds control logic and public interface for the elevator. */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged replayedInputs = new ClimbIOInputsAutoLogged();

  private InnerClimbHeight currentInnerClimbSetpoint = InnerClimbHeight.DOWN;
  private OuterClimbHeight currentOuterClimbSetpoint = OuterClimbHeight.DOWN;
  private Voltage motorVoltage = Volt.of(3);
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
        "Climb/Inner Climb/Carriage Setpoint (inches)",
        currentInnerClimbSetpoint.getInnerPosition().in(Inches));
    Logger.recordOutput(
        "Climb/Inner Climb/Motor Setpoint (rotations)",
        currentInnerClimbSetpoint.getInnerPositionAngle().in(Rotations));
    Logger.recordOutput(
        "Climb/Outer Climb/Carriage Setpoint (inches)",
        currentOuterClimbSetpoint.getOuterPosition().in(Inches));
    Logger.recordOutput(
        "Climb/Outer Climb/Motor Setpoint (rotations)",
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

  public void setInnerMotorVoltage() {
    io.setInnerMotorVoltage(motorVoltage);
  }

  public void setOuterMotorVoltage(Voltage motorVoltage) {
    io.setOuterMotorVoltage(motorVoltage);
  }

  public Command setInnerClimbPositionCommand(InnerClimbHeight height) {
    return new InstantCommand(() -> setInnerClimbPosition(height));
  }

  public Command setOuterClimbPositionCommand(OuterClimbHeight height) {
    return new InstantCommand(() -> setOuterClimbPosition(height));
  }

  public Command manuelOuterClimb(double value) {
    Voltage motorVoltage = Volt.of(3 * value);
    return new InstantCommand(() -> setOuterMotorVoltage(motorVoltage));
  }

  public Command manuelInnerClimb() {
    return new InstantCommand(() -> setInnerMotorVoltage());
  }

  public Command postAutoClimb() {
    return setInnerClimbPositionCommand(InnerClimbHeight.POSTAUTO);
  }

  public Command deployClimb() {
    return new SequentialCommandGroup(
        setOuterClimbPositionCommand(OuterClimbHeight.UP),
        setInnerClimbPositionCommand(InnerClimbHeight.UP));
  }

  public Command l1Sequence() {
    return setInnerClimbPositionCommand(InnerClimbHeight.DOWN);
  }

  public Command l2Sequence() {
    return new SequentialCommandGroup(
        l1Sequence(),
        setOuterClimbPositionCommand(OuterClimbHeight.DOWN),
        new WaitCommand(1),
        setInnerClimbPositionCommand(InnerClimbHeight.UP),
        new WaitCommand(1),
        setInnerClimbPositionCommand(InnerClimbHeight.DOWN),
        new WaitCommand(1),
        setOuterClimbPositionCommand(OuterClimbHeight.UP));
  }

  public Command l3Sequence() {
    return new SequentialCommandGroup(
        l2Sequence(),
        setOuterClimbPositionCommand(OuterClimbHeight.DOWN),
        new WaitCommand(1),
        setInnerClimbPositionCommand(InnerClimbHeight.UP),
        new WaitCommand(1),
        setInnerClimbPositionCommand(InnerClimbHeight.DOWN),
        new WaitCommand(1),
        setOuterClimbPositionCommand(OuterClimbHeight.UP));
  }

  public enum InnerClimbHeight {
    // elliot said add 3 inches since its not a normal elevator beacuse a rope is spolling it,
    // except for down
    // Origional values UP(Inches.of(9.75)), MIDDLE(Inches.of(4.875)),
    UP(Inches.of(12.75)),
    // TODO figure post auto position
    POSTAUTO(Inches.of(4)),
    MIDDLE(Inches.of(7.875)),
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
          position.in(Inches)
              / ClimbConstants.INNER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Inches));
    }
  }

  public enum OuterClimbHeight {
    // elliot said add 3 inches since its not a normal elevator beacuse a rope is spolling it,
    // except for down
    // Origional values UP(Inches.of(11)), MIDDLE(Inches.of(5.5)),
    UP(Inches.of(14)),
    MIDDLE(Inches.of(8.5)),
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
          position.in(Inches)
              / ClimbConstants.OUTER_CLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION.in(Inches));
    }
  }
}
