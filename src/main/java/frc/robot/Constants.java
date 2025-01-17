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

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double loopPeriodSecs = Robot.defaultPeriodSecs;

  public static final class AdvantageKitConstants {
    public static enum Mode {
      REAL,
      REPLAY,
      SIM
    }

    private static Mode kfakeMode = Mode.SIM;

    public static Mode getMode() {
      return RobotBase.isReal() ? Mode.REAL : kfakeMode;
    }
  }

  public static final class PathPlannerConstants {
    public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

    public static final double kMaxAngularAcceleration = 4 * Math.PI; // This was made up
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.00; // This was made up

    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(
            DriveConstants.maxSpeedMetersPerSec,
            PathPlannerConstants.kMaxAccelerationMetersPerSecondSquared,
            DriveConstants.maxSpeedMetersPerSec,
            5 * Math.PI);

    public static final PathConstraints TEST_PATH_CONSTRAINTS =
        new PathConstraints(
            1.0,
            PathPlannerConstants.kMaxAccelerationMetersPerSecondSquared,
            DriveConstants.maxSpeedMetersPerSec,
            5 * Math.PI);
  }
}
