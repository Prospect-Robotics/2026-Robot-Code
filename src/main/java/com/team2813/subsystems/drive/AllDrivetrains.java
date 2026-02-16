package com.team2813.subsystems.drive;

import com.team2813.generated.drwomp.TunerConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Map;
import java.util.function.Supplier;

public class AllDrivetrains {
  private static final String SIMULATOR_SERIAL_NUM = "";
  private static final String DR_WOMP_SERIAL_NUM = "327B9D0";
  private static final String REBUILT_SERIAL_NUM = "24056E5";

  private static final Map<String, Supplier<AllTunerConstants>> SERIAL_NUMBER_TO_TUNER_CONSTANTS =
      Map.of(
          SIMULATOR_SERIAL_NUM,
          AllDrivetrains::defaultDrivetrain,
          DR_WOMP_SERIAL_NUM,
          AllDrivetrains::drWomp);

  public static AllTunerConstants drWomp() {
    AllTunerConstants robotConstants =
        new AllTunerConstants(
            "Dr. Womp",
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight,
            TunerConstants.DrivetrainConstants,
            TunerConstants.kCANBus,
            TunerConstants.kSpeedAt12Volts);
    return robotConstants;
  }

  private static AllTunerConstants defaultDrivetrain() {
    // TODO: Return constants for the 2026 robot.
    return drWomp();
  }

  public static AllTunerConstants forRoboRIO() {
    String sNumber = RobotController.getSerialNumber();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Metadata");
    table.getStringTopic("RoboRIO Serial Number").publish().set(sNumber);

    var tunerConstantsSupplier = SERIAL_NUMBER_TO_TUNER_CONSTANTS.get(sNumber);
    if (tunerConstantsSupplier == null) {
      DriverStation.reportError(
          String.format("no tuner constants for serial number '%s'", sNumber), false);
      return defaultDrivetrain();
    }
    AllTunerConstants tunerConstants = tunerConstantsSupplier.get();
    table.getStringTopic("Drivetrain").publish().set(tunerConstants.drivetrainName());
    return tunerConstants;
  }

  private AllDrivetrains() {
    throw new AssertionError("Not instantiable");
  }
}
