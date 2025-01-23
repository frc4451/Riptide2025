package frc.robot.field;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.bobot_state.BobotState;

public class FieldUtils {
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
  }

  public static boolean isBlueAlliance() {
    return FieldUtils.getAlliance() == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return FieldUtils.getAlliance() == Alliance.Red;
  }

  public static double getFlipped() {
    return FieldUtils.isRedAlliance() ? -1 : 1;
  }

  public static HPSZone getHPSZone() {
    // Relative to the blue alliance half field
    HPSZone zone =
        BobotState.getGlobalPose().getY() > FieldConstants.halfFieldWidth
            ? HPSZone.DRIVER_LEFT
            : HPSZone.DRIVER_RIGHT;

    // Flip if Red Alliance
    if (FieldUtils.isRedAlliance()) {
      zone = (zone == HPSZone.DRIVER_LEFT) ? HPSZone.DRIVER_RIGHT : HPSZone.DRIVER_LEFT;
    }

    return zone;
  }

  public static Rotation2d getHPSOffset() {
    // This is entirely for visual demonstration. You should rewrite this to use VARC
    return FieldUtils.getHPSZone() == HPSZone.DRIVER_LEFT
        ? Rotation2d.fromDegrees(60)
        : Rotation2d.fromDegrees(300);
  }
}
