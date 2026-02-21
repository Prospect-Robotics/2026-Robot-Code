package com.team2813.subsystems.intakeroller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team2813.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  // private static final int HOPPER_FUEL_CAPACITY = 46;

  private TalonFX intakeMotor;
  private TalonFXSimState intakeSimState;

  private FlywheelSim intakeFlywheelSim;
  // See
  // https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/#over-the-bumper-otb-intakes
  // for description on how this intake simulation is set up.
  // private final IntakeSimulation intakeSimulation;

  BooleanConsumer runFuelIntake = (intakeActivated) -> {};

  public IntakeRollerIOSim(BooleanConsumer runFuelIntake) {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);
    intakeMotor.getConfigurator().apply(IntakeRollerConstants.INTAKE_MOTOR_CONFIG);

    intakeSimState = intakeMotor.getSimState();

    intakeFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                IntakeRollerConstants.INTAKE_SIM_MOI,
                IntakeRollerConstants.INTAKE_MOTOR_TO_INTAKE_GEARING),
            DCMotor.getKrakenX60(1));

    this.runFuelIntake = runFuelIntake;

    // intakeSimulation =
    //     IntakeSimulation.OverTheBumperIntake(
    //         // Specify the type of game pieces that the intake can collect
    //         "Fuel",
    //         // Specify the drivetrain to which this intake is attached
    //         driveTrainSimulation,
    //         // Width of the intake
    //         Meters.of(0.7), // TODO(vdikov): Confirm in CAD
    //         // The extension length of the intake beyond the robot's frame (when activated)
    //         Meters.of(0.2), // TODO(vdikov): Confirm in CAD
    //         // The intake is mounted on the back side of the chassis
    //         IntakeSimulation.IntakeSide.FRONT,
    //         // The intake can hold up to 1 note
    //         HOPPER_FUEL_CAPACITY);
  }

  @Override
  public void updateState(IntakeRollerIOInputs inputs) {
    updateSimulation();

    intakeSimState.setSupplyVoltage(Volts.of(12));

    inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValue();
    inputs.intakeMotorRPS = intakeMotor.getVelocity().getValue();
    inputs.intakeMotorCurrent = intakeMotor.getStatorCurrent().getValue();

    // Logger.recordOutput("IntakeRoller/Sim/IntakeHasFuel",
    // intakeSimulation.getGamePiecesAmount());
  }

  public void updateSimulation() {
    // TODO: Once we fetch from main, change this to the SIM_TIME in Constants.
    intakeFlywheelSim.update(0.02);

    intakeSimState.setRotorAcceleration(intakeFlywheelSim.getAngularAcceleration());
    intakeSimState.setRotorVelocity(intakeFlywheelSim.getAngularVelocity());
  }

  @Override
  public void setIntakeMotorVoltage(Voltage intakeMotorVoltage) {
    // TODO(vdikov): Actually, pay attention to the voltage sign when determining whether to signal
    // running intake.
    runFuelIntake.accept(Math.abs(intakeMotorVoltage.in(Volts)) > 0.5);

    intakeMotor.setVoltage(intakeMotorVoltage.in(Volts));
    intakeFlywheelSim.setInputVoltage(intakeMotorVoltage.in(Volts));
  }
}
