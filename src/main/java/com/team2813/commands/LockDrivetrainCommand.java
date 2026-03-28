package com.team2813.commands;

import com.team2813.subsystems.drive.Drive;
import com.team2813.util.HubPositionUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LockDrivetrainCommand extends Command {
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier faceHub;
  private final Drive drive;
  private boolean crossed;
  private final Command facingCommand;
  private final Command normalCommand;

  public LockDrivetrainCommand(
      Drive drive,
      DoubleSupplier vxSupplier,
      DoubleSupplier vySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier faceHub,
      Optional<DriverStation.Alliance> alliance) {
    this.drive = drive;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.omegaSupplier = omegaSupplier;

    this.faceHub = faceHub;

    facingCommand =
        DriveCommands.joystickDriveAtAngle(
            drive,
            vxSupplier,
            vySupplier,
            () -> HubPositionUtil.getBotToHubAngle(drive.getPose(), alliance));

    normalCommand = DriveCommands.joystickDrive(drive, vxSupplier, vySupplier, omegaSupplier);
  }

  @Override
  public void initialize() {
    crossed = false;
  }

  @Override
  public void execute() {
    double vx = vxSupplier.getAsDouble();
    double vy = vySupplier.getAsDouble();
    double omega = vxSupplier.getAsDouble();

    if (MathUtil.isNear(0, vx, 0.1)
        || MathUtil.isNear(0, vy, 0.1)
        || MathUtil.isNear(0, omega, 0.1)) {
      // we have a requested movement
      crossed = false;
      if (faceHub.getAsBoolean()) {
        facingCommand.execute();
      } else {
        normalCommand.execute();
      }
    } else if (!crossed) {
      drive.stopWithX();
      crossed = true;
    }
  }
}
