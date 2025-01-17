package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.vision.VisionConstants;

public class GarageUtils {
  /** Simpler way to get current alliance, or return our predetermined "DEFAULT" alliance. */
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().isPresent()
        ? DriverStation.getAlliance().get()
        : PathPlannerConstants.DEFAULT_ALLIANCE;
  }

  public static boolean isBlueAlliance() {
    return GarageUtils.getAlliance() == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return GarageUtils.getAlliance() == Alliance.Red;
  }

  public static double getFlipped() {
    return GarageUtils.isRedAlliance() ? -1 : 1;
  }

  public static double percentWithSoftStops(
      double percentDecimal, double position, double min, double max) {
    boolean canMoveUp = (percentDecimal > 0.0 && position < max);
    boolean canMoveDown = (percentDecimal < 0.0 && position > min);
    return (canMoveUp || canMoveDown) ? percentDecimal : 0.0;
  }

  public static int getSpeakerTag() {
    return GarageUtils.isBlueAlliance()
        ? VisionConstants.BLUE_SPEAKER_CENTER
        : VisionConstants.RED_SPEAKER_CENTER;
  }
}
