package com.team2813.subsystems.drive;

import com.team2813.generated.drwomp.TunerConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Map;

public class AllDrivetrains {
  static Map<String, AllTunerConstants> robotConstants = Map.of("", drWomp());

  public static AllTunerConstants drWomp() {
    AllTunerConstants robotConstants =
        new AllTunerConstants(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    return robotConstants;
  }

  public static AllTunerConstants forRoboRIO() {
    String sNumber = RobotController.getSerialNumber();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Metadata");
    table.getStringTopic("RoboRIO Serial Number").publish().set(sNumber);
    var tunerConstants = robotConstants.get(sNumber);
    if (tunerConstants == null) {
      // tunerConstants = drWomp();
      DriverStation.reportError("no tuner constants for serial number", null);
    }
    return tunerConstants;
  }
}
