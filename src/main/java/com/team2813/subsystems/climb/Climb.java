package com.team2813.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private ClimbHeight currentClimbSetpoint = ClimbHeight.DOWN;

  /**
   * @param io The hardware implementation for the elevator, either sim or real.
   */
  public Climb(ClimbIO io) {
    var slot0Config =
        new Slot0Configs() // Motor PID and gain values.
            .withKP(ClimbConstants.LEFTCLIMB_kP)
            .withKI(ClimbConstants.LEFTCLIMB_kI)
            .withKD(ClimbConstants.LEFTCLIMB_kD)
            .withKS(ClimbConstants.LEFTCLIMB_kS)
            .withKV(ClimbConstants.LEFTCLIMB_kV)
            .withKA(ClimbConstants.LEFTCLIMB_kA)
            .withKG(ClimbConstants.LEFTCLIMB_kG);
    var motorConfig =
        new TalonFXConfiguration()
            .withSlot0(slot0Config)
            // .withMotorOutput(new
            // MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
            // Invert motor rotation.
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    var motor = new TalonFX(com.team2813.Constants.LEFTCLIMB_MOTOR_ID);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(motorConfig);

    io.setMotor(motor);
    this.io = io;
  }

  public void stopClimb() {
    io.setMotorVoltage(Volts.of(0));
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
        "Climb/Carriage Setpoint (inches)", currentClimbSetpoint.getPosition().in(Inches));
    Logger.recordOutput(
        "Climb/Motor Setpoint (rotations)", currentClimbSetpoint.getPositionAngle().in(Rotations));
  }

  @Override
  public void simulationPeriodic() {
    com.team2813.subsystems.SimulationVisualizer.getInstance()
        .updateElevatorHeight(Inches.of(replayedInputs.leftCarriagePositionInches));
  }

  public void setClimbPosition(ClimbHeight heightSetpoint) {
    currentClimbSetpoint = heightSetpoint;
    io.setMotorSetpoint(heightSetpoint.getPositionAngle());
  }

  public Command setClimbPositionCommand(ClimbHeight height) {
    return new InstantCommand(() -> setClimbPosition(height));
  }

  public enum ClimbHeight {
    // Positions taken from offseason bot code, inturn taken from onshape.
    UP(Inches.of(56.0)),
    MIDDLE(Inches.of(28.0)),
    DOWN(Inches.of(0.0));

    public final Distance position;

    ClimbHeight(Distance position) {
      this.position = position;
    }

    public Distance getPosition() {
      return position;
    }

    public Angle getPositionAngle() {
      // NOTE: Divide by 2 because the motor controls the first stage only, not the second stage
      return Rotations.of(
          (getPosition().in(Inches) / 2)
              / ClimbConstants.INNERCLIMB_HEIGHT_CHANGE_PER_MOTOR_ROTATION);
    }
  }
}
