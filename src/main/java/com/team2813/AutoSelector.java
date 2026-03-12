package com.team2813;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

interface AutoSelector {
  Command getAutonomousCommand();

  void addOption(String name, Supplier<Command> generator);
}
