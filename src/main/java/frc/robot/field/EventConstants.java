package frc.robot.field;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
      Units.inchesToMeters(23.5),
      Units.inchesToMeters(24.0),
      Units.inchesToMeters(44.0),
      Units.inchesToMeters(40.0),
      new Transform2d(
          Units.inchesToMeters(22), Units.inchesToMeters(2), Rotation2d.fromDegrees(180 + 30)),
      new Transform2d(
          Units.inchesToMeters(22), Units.inchesToMeters(-1), Rotation2d.fromDegrees(180 - 30))),
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
      Double.NaN,
      Double.NaN,
      Transform2d.kZero,
      Transform2d.kZero),
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
      Double.NaN,
      Double.NaN,
      Transform2d.kZero,
      Transform2d.kZero),
  WORLDS(
      Units.inchesToMeters(-6.5),
      Units.inchesToMeters(6.5),
      Units.inchesToMeters(17.0),
      Units.inchesToMeters(18.5),
      Units.inchesToMeters(25.0),
      Units.inchesToMeters(18.5),
      Units.inchesToMeters(25.0),
      Units.inchesToMeters(23.5),
      Units.inchesToMeters(24.0),
      Units.inchesToMeters(44.0),
      Units.inchesToMeters(40.0),
      new Transform2d(
          Units.inchesToMeters(22), Units.inchesToMeters(2), Rotation2d.fromDegrees(180 + 30)),
      new Transform2d(
          Units.inchesToMeters(22), Units.inchesToMeters(-1), Rotation2d.fromDegrees(180 - 30)));

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

  public final double bargeShotOffset;

  public final Transform2d leftL1;
  public final Transform2d rightL1;

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
      double cageSideOffset,
      double bargeShotOffset,
      Transform2d leftL1,
      Transform2d rightL1) {
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
    this.bargeShotOffset = bargeShotOffset;
    this.leftL1 = leftL1;
    this.rightL1 = rightL1;
  }
}
