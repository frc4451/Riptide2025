package frc.robot.auto;

import edu.wpi.first.math.util.Units;

public class AutoConstants {
  public static final double l2ReefOffsetMeters = Units.inchesToMeters(16.0);
  public static final double l4ReefOffsetMeters = Units.inchesToMeters(19.0);
  public static final double elevatorDownOffsetMeters = Units.inchesToMeters(25.0);

  public static final double l2RumbleDistanceMters = l2ReefOffsetMeters;
  public static final double l4RumbleDistanceMters = l4ReefOffsetMeters;

  public static final boolean useConstrainedPoseForReef = false;
}
