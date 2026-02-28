package com.team2813.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged replayedInputs;

  // When either button (on elastic) is pushed, the voltage will either increase or decrease by 0.5
  private final static String TRENCH_VOLTAGE_INCREASE_BUTTON_KEY = "TRENCH_SHOOTER_VOLTAGE_INCREASE";
  private final static String TRENCH_VOLTAGE_DECREASE_BUTTON_KEY = "TRENCH_SHOOTER_VOLTAGE_DECREASE";

  // Shows the current set voltage
  private final static String TRENCH_CURRENT_VOLTAGE_REPORTER_KEY = "TRENCH_SHOOTER_CURRENT_VOLTAGE";

  // double in order to save on object initialization
  private double currentTrenchShooterVoltage = 11;

  // When either button (on elastic) is pushed, the voltage will either increase or decrease by 0.5
  private final static String HUB_VOLTAGE_INCREASE_BUTTON_KEY = "HUB_SHOOTER_VOLTAGE_INCREASE";
  private final static String HUB_VOLTAGE_DECREASE_BUTTON_KEY = "HUB_SHOOTER_VOLTAGE_DECREASE";

  private final static String HUB_CURRENT_VOLTAGE_REPORTER_KEY = "HUB_SHOOTER_CURRENT_VOLTAGE";

  private double currentHubShooterVoltage = 5.5;

  public Shooter(ShooterIO io) {
    this.io = io;
    this.replayedInputs = new ShooterIOInputsAutoLogged();



  }

  /**
   * Increase the voltage for a trench shot by 0.5.
   */
  private void increaseTrenchShooterVoltage() {
    if (currentHubShooterVoltage <= 13)  { // Max voltage is 13.
      currentTrenchShooterVoltage += 0.5;
    }
  }

  /**
   * Decrease the voltage for a trench shot by 0.5.
   */
  private void decreaseTrenchShooterVoltage() {
    if (currentTrenchShooterVoltage >= 0)  { // Min voltage is 0
      currentTrenchShooterVoltage -= 0.5;
    }
  }

  /**
   * Increase the voltage for a hub shot by 0.5.
   */
  private void increaseHubShooterVoltage() {
    if (currentHubShooterVoltage <= 13) {
      currentHubShooterVoltage += 0.5;
    }
  }

  /**
   * Decrease the voltage for a hub shot by 0.5.
   */
  private void decreaseHubShooterVoltage() {
    if  (currentHubShooterVoltage >= 0)  {
      currentHubShooterVoltage -= 0.5;
    }
  }

  @Override
  public void periodic() {
    io.updateState(replayedInputs);

    Logger.processInputs("Shooter", replayedInputs);

    Logger.recordOutput("Shooter/currentTrenchShooterVoltage", currentTrenchShooterVoltage);
    Logger.recordOutput("Shooter/currentHubShooterVoltage", currentHubShooterVoltage);


    SmartDashboard.updateValues();
  }

  public void stop() {
    io.setShooterMotorVoltage(Volts.of(0));
  }

  /**
   * @return A {@link StartEndCommand} that spools the shooter to shoot the fuel.
   */
  public Command spoolShooterCommand() {
    return new StartEndCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterIntakeVoltage()),
        this::stop,
        this);
  }

  public Command outakeCommand() {
    return new StartEndCommand(
        () -> io.setShooterMotorVoltage(ShooterConstants.getShooterOuttakeVoltage()), this::stop);
  }

  // Instructions taken from https://docs.advantagekit.org/data-flow/sysid-compatibility/ and
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
  public Command sysIDRoutine() {
    SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIDTestState", state.toString())),
            new SysIdRoutine.Mechanism(io::setShooterMotorVoltage, null, this));
    // NOTE(spderman3333): I may need to use this::setShooterMotorVoltage rather than
    // io::setShooterMotorVoltage.

    return new SequentialCommandGroup(
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  // Used for auto calculated motor speed.
  public void setShooterMotorVoltage(Voltage voltage) {
    io.setShooterMotorVoltage(voltage);
  }
}
