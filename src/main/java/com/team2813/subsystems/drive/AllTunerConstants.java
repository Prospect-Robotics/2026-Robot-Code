package com.team2813.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * A record class that holds tuner constants.
 *
 * @param frontLeft The swerve constants for the front-left module.
 * @param frontRight The swerve constants for the front-right module.
 * @param backLeft The swerve constants for the back-left module.
 * @param backRight The swerve constants for the back-right module.
 */
public record AllTunerConstants(
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        frontLeft,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        frontRight,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        backLeft,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        backRight,
    SwerveDrivetrainConstants drivetrainConstants,
    CANBus CANBus,
    LinearVelocity kSpeedAt12Volts) {}
