package com.team2813.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final ClimbIO io;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public void climbOuterHooks() {}

  public Command l3_Climb_Sequnce() {
    return new InstantCommand();
  }

  public Command deploy_Climb() {
    return new InstantCommand();
  }

  public Command l1_Climb_Sequnce() {
    return new InstantCommand();
  }

  public Command l2_Climb_Sequnce() {
    return new InstantCommand();
  }

  public Command post_Auto_Climb_Sequence() {
    return new InstantCommand();
  }

  public Command retract_Passive_Hooks() {
    return new InstantCommand();
  }

  public Command extend_Passive_Hooks() {
    return new InstantCommand();
  }
}
