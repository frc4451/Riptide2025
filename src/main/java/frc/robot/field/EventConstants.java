package frc.robot.field;

import edu.wpi.first.math.util.Units;

public enum EventConstants {
  HOME(
      Units.inchesToMeters(-6.5),
      Units.inchesToMeters(6.5),
      Units.inchesToMeters(17.0),
      Units.inchesToMeters(21.0),
      Units.inchesToMeters(25.0),
      Units.inchesToMeters(20.0),
      Units.inchesToMeters(25.0),
      Units.inchesToMeters(30.0),
      Units.inchesToMeters(24.0),
      Units.inchesToMeters(44.0)),
  NORTH_CHARLESTON(
      Units.inchesToMeters(-7.0),
      Units.inchesToMeters(8.0),
      Units.inchesToMeters(16.0),
      Units.inchesToMeters(19.0),
      Units.inchesToMeters(25.0),
      Units.inchesToMeters(17.0),
      Units.inchesToMeters(25.0),
      Double.NaN,
      Double.NaN,
      Double.NaN),
  DCMP(
      Units.inchesToMeters(-6.5),
      Units.inchesToMeters(6.5),
      Units.inchesToMeters(17.0),
      Units.inchesToMeters(18.5),
      Units.inchesToMeters(25.0),
      Units.inchesToMeters(18.0),
      Units.inchesToMeters(25.0),
      Double.NaN,
      Double.NaN,
      Double.NaN);
  /** Distance from the center of the April Tag on the Face to the center of the Pole */
  public final double tagToReefLeft;

  /** Distance from the center of the April Tag on the Face to the center of the Pole */
  public final double tagToReefRight;

  public final double l2ReefOffset;
  public final double l2RumbleDistance;

  public final double l4ReefOffset;
  public final double l4RumbleDistance;

  public final double elevatorDownOffset;

  public final double hpsOffset;
  public final double hpsSideOffset;

  public final double algaeOffset;

  public final double cageBackOffset;
  public final double cageSideOffset;

  private EventConstants(
      double tagToReefLeft,
      double tagToReefRight,
      double l2ReefOffsetMeters,
      double l4ReefOffsetMeters,
      double elevatorDownOffsetMeters,
      double hpsOffsetMeters,
      double hpsSideOffsetMeters,
      double algaeOffsetMeters,
      double cageBackOffset,
      double cageSideOffset) {
    this.tagToReefLeft = tagToReefLeft;
    this.tagToReefRight = tagToReefRight;
    this.l2ReefOffset = l2ReefOffsetMeters;
    this.l2RumbleDistance = l2ReefOffsetMeters + Units.inchesToMeters(1);
    this.l4ReefOffset = l4ReefOffsetMeters;
    this.l4RumbleDistance = l4ReefOffsetMeters + Units.inchesToMeters(1);
    this.elevatorDownOffset = elevatorDownOffsetMeters;
    this.hpsOffset = hpsOffsetMeters;
    this.hpsSideOffset = hpsSideOffsetMeters;
    this.algaeOffset = algaeOffsetMeters;
    this.cageBackOffset = cageBackOffset;
    this.cageSideOffset = cageSideOffset;
  }
}
