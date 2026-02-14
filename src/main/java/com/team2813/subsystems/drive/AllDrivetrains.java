package com.team2813.subsystems.drive;

import com.team2813.generated.drwomp.TunerConstants;

public class AllDrivetrains {

  public static AllTunerConstants drWomp() {
    AllTunerConstants robotConstants =
        new AllTunerConstants(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight,
            TunerConstants.DrivetrainConstants,
            TunerConstants.kCANBus,
            TunerConstants.kSpeedAt12Volts);
    return robotConstants;
  }
}
