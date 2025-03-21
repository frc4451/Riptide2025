package frc.robot.auto;

import edu.wpi.first.math.util.Units;

public class AutoConstants {
  public static final double l2ReefOffsetMeters = Units.inchesToMeters(16.0);
  public static final double l4ReefOffsetMeters = Units.inchesToMeters(19.0);
  public static final double elevatorDownOffsetMeters = Units.inchesToMeters(25.0);

  public static final double l2RumbleDistanceMters = Units.inchesToMeters(21.6);
  public static final double l4RumbleDistanceMters = Units.inchesToMeters(20.0);

  public static final boolean useConstrainedPoseForReef = false;

  public static final double l2MinDistanceMeters = Units.inchesToMeters(15.0);
  public static final double l2MaxDistanceMeters = Units.inchesToMeters(16.0);

  public static final double l4MinDistanceMeters = Units.inchesToMeters(14.0);
  public static final double l4MaxDistanceMeters = Units.inchesToMeters(19.0);
}
