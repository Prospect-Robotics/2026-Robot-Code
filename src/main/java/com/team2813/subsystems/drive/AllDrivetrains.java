package com.team2813.subsystems.drive;

import com.team2813.generated.jones.TunerConstants;

public class AllDrivetrains {

  public static AllTunerConstants drWomp() {
    AllTunerConstants robotConstants =
        new AllTunerConstants(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    return robotConstants;
  }
}
