package frc.robot.field;

import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX = Units.inchesToMeters(299.438);

  /** Used for calculating HPS zoning */
  public static final double halfFieldWidth = fieldWidth / 2;

  /*
   * April Tag Lookup
   */
  public static final Integer blueHPSDriverRight = 12;
  public static final Integer blueHPSDriverLeft = 13;
  public static final Integer blueProcessor = 16;
  public static final Integer blueBarge = 14; // TODO how do we want to name this?

  // See Allred's sheet for why they're AB/CD/EF/etc
  public static final Integer blueReefAB = 18;
  public static final Integer blueReefCD = 17;
  public static final Integer blueReefEF = 22;
  public static final Integer blueReefGH = 21;
  public static final Integer blueReefIJ = 20;
  public static final Integer blueReefKL = 19;

  public static final Integer redHPSDriverLeft = 1;
  public static final Integer redHPSDriverRight = 2;

  public static final Integer redProcessor = 3;
  public static final Integer redBarge = 4;

  public static final Integer redReefAB = 7;
  public static final Integer redReefCD = 8;
  public static final Integer redReefEF = 9;
  public static final Integer redReefGH = 10;
  public static final Integer redReefIJ = 11;
  public static final Integer redReefKL = 6;
}
