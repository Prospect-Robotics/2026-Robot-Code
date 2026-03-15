package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Class that holds control logic and public interface for the elevator.
 *
 * @param <T> An distance supplier used for motor setpoints.
 */
public class Climb<T extends Supplier<Distance>> extends SubsystemBase {
  private final ClimbIO io;
  private final String name;
  private final ClimbIOInputsAutoLogged replayedInputs = new ClimbIOInputsAutoLogged();

  private Angle currentClimbSetpointRotations = Rotations.of(0);
  private Distance currentClimbSetpointInches = Inches.of(0);

  /**
   * @param io The hardware implementation for the climb, either sim or real.
   */
  public Climb(ClimbIO io) {
    this.io = io;
    this.name = io.climbConstants.climbName();
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);
    // `processInputs` must be called every periodic after updating hardware state.
    // In `REPLAY` mode, `updateState` does nothing, and the `replayedInputs` are populated from the
    // replayed logs
    // instead.
    Logger.processInputs("Climb/" + name, replayedInputs);
    Logger.recordOutput("Climb/" + name + "/Carriage Setpoint (inches)", currentClimbSetpointInches.in(Inches));
    Logger.recordOutput("Climb/" + name + "/Motor Setpoint (rotations)",
        currentClimbSetpointRotations.in(Rotations));
  }

  @Override
  public void simulationPeriodic() {}

  public void stopClimb() {
    io.stopMotor();
  }

  public void setClimbPosition(T heightSetpoint) {
    currentClimbSetpointInches = heightSetpoint.get();
    currentClimbSetpointRotations = convertExtenderHeightToMotorAngle(heightSetpoint);
    io.setMotorSetpoint(currentClimbSetpointRotations);
  }

  public void setMotorVoltage(Voltage motorVoltage) {
    io.setMotorVoltage(motorVoltage);
  }

  public Command setClimbPositionCommand(T heightSetpoint) {
    return new InstantCommand(() -> setClimbPosition(heightSetpoint));
  }

  private Angle convertExtenderHeightToMotorAngle(T heightPositionSetpoint) {
    return Rotations.of(
        heightPositionSetpoint.get().in(Inches)
            / io.climbConstants.climbHeightChangePerRotation().in(Inches));
  }

  public boolean atSetpointPosition() {
    return io.getMotorPosition()
        .isNear(
            currentClimbSetpointRotations, ClimbConstants.CLIMB_SETPOINT_TO_MOTOR_ROT_TOLERANCE);
  }

  // TODO: Move these into a different class as we split climb into two instances.
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
  //
}
