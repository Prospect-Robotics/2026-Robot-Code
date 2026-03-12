package com.team2813;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class PathPlannerAutos implements AutoSelector {
  private final LoggedDashboardChooser<Command> autoChooser;

  public PathPlannerAutos() {
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public void addOption(String name, Supplier<Command> generator) {
    autoChooser.addOption(name, generator.get());
  }
}
