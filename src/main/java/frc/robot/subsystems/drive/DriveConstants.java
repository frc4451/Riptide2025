// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
  // TODO: max speed, wheel radius, gyro trimming
  public static final double maxSpeedMetersPerSec = 4.4;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(Constants.isGuido ? 20.5 : 22.5);
  public static final double wheelBase = trackWidth;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module.
  // When calibrating, hold the metal bar to each wheel,
  // using the flat side (without gear) to the right of the robot
  public static final Rotation2d frontLeftZeroRotation =
      new Rotation2d(Constants.isGuido ? -1.5332492033587857 : -1.9194672743426724);
  public static final Rotation2d frontRightZeroRotation =
      new Rotation2d(Constants.isGuido ? -3.137842957173483 : 1.587 + Math.PI);
  public static final Rotation2d backLeftZeroRotation =
      new Rotation2d(Constants.isGuido ? -3.138901297246115 : 5.186 + Math.PI);
  public static final Rotation2d backRightZeroRotation =
      new Rotation2d(Constants.isGuido ? 1.5813666582107544 : 0.11636602878570557);

  public static final double gyroTrimScalar = Constants.isGuido ? 1.501 : 0.0;

  // Device CAN IDs
  public static final int pigeonCanId = 1;
  public static final double mountPoseYawDeg = Constants.isGuido ? -0.9988248944282532 : 0;
  public static final double mountPosePitchDeg = Constants.isGuido ? -0.2277984470129013 : 0;
  public static final double mountPoseRollDeg = Constants.isGuido ? 1.5160924196243286 : 0;

  public static final int frontLeftDriveCanId = Constants.isGuido ? 15 : 13;
  public static final int frontRightDriveCanId = Constants.isGuido ? 11 : 11;
  public static final int backLeftDriveCanId = Constants.isGuido ? 17 : 17;
  public static final int backRightDriveCanId = Constants.isGuido ? 13 : 15;

  public static final int frontLeftTurnCanId = Constants.isGuido ? 14 : 16;
  public static final int frontRightTurnCanId = Constants.isGuido ? 10 : 14;
  public static final int backLeftTurnCanId = Constants.isGuido ? 16 : 10;
  public static final int backRightTurnCanId = Constants.isGuido ? 12 : 12;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.4797244566677281);
  public static final double driveMotorReduction =
      (45.0 * 22.0)
          / ((Constants.isGuido ? 12.0 : 13.0)
              * 15.0); // MAXSwerve with 12 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox =
      Constants.isGuido ? DCMotor.getNEO(1) : DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.13014;
  public static final double driveKv = 0.09750;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
}
