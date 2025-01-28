package frc.robot.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.List;

public class FieldConstants {
  /** AdvantageKit-safe loggable version of `AprilTag` that contains data we want without lookups */
  public static record AprilTagStruct(int fiducialId, Pose3d pose) {}

  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX = Units.inchesToMeters(299.438);

  /** Used for calculating HPS zoning */
  public static final double halfFieldWidth = fieldWidth / 2;

  /*
   * April Tag Lookup
   */
  public static final AprilTagStruct blueHPSDriverRight =
      new AprilTagStruct(12, VisionConstants.fieldLayout.getTagPose(12).get());
  public static final AprilTagStruct blueHPSDriverLeft =
      new AprilTagStruct(13, VisionConstants.fieldLayout.getTagPose(13).get());

  public static final Integer blueProcessor = 16;
  public static final Integer blueBarge = 14; // TODO how do we want to name this?

  // See Allred's sheet for why they're AB/CD/EF/etc
  public static final AprilTagStruct blueReefAB =
      new AprilTagStruct(18, VisionConstants.fieldLayout.getTagPose(18).get());
  public static final AprilTagStruct blueReefCD =
      new AprilTagStruct(17, VisionConstants.fieldLayout.getTagPose(17).get());
  public static final AprilTagStruct blueReefEF =
      new AprilTagStruct(22, VisionConstants.fieldLayout.getTagPose(22).get());
  public static final AprilTagStruct blueReefGH =
      new AprilTagStruct(21, VisionConstants.fieldLayout.getTagPose(21).get());
  public static final AprilTagStruct blueReefIJ =
      new AprilTagStruct(20, VisionConstants.fieldLayout.getTagPose(20).get());
  public static final AprilTagStruct blueReefKL =
      new AprilTagStruct(19, VisionConstants.fieldLayout.getTagPose(19).get());

  public static final AprilTagStruct redHPSDriverLeft =
      new AprilTagStruct(1, VisionConstants.fieldLayout.getTagPose(1).get());
  public static final AprilTagStruct redHPSDriverRight =
      new AprilTagStruct(2, VisionConstants.fieldLayout.getTagPose(2).get());

  public static final Integer redProcessor = 3;
  public static final Integer redBarge = 4;

  public static final AprilTagStruct redReefAB =
      new AprilTagStruct(7, VisionConstants.fieldLayout.getTagPose(7).get());
  public static final AprilTagStruct redReefCD =
      new AprilTagStruct(8, VisionConstants.fieldLayout.getTagPose(8).get());
  public static final AprilTagStruct redReefEF =
      new AprilTagStruct(9, VisionConstants.fieldLayout.getTagPose(9).get());
  public static final AprilTagStruct redReefGH =
      new AprilTagStruct(10, VisionConstants.fieldLayout.getTagPose(10).get());
  public static final AprilTagStruct redReefIJ =
      new AprilTagStruct(11, VisionConstants.fieldLayout.getTagPose(11).get());
  public static final AprilTagStruct redReefKL =
      new AprilTagStruct(6, VisionConstants.fieldLayout.getTagPose(6).get());

  public static final List<AprilTagStruct> blueReefTags =
      List.of(blueReefAB, blueReefCD, blueReefEF, blueReefGH, blueReefIJ, blueReefKL);
  public static final List<AprilTagStruct> redReefTags =
      List.of(redReefAB, redReefCD, redReefEF, redReefGH, redReefIJ, redReefKL);

  public static final List<AprilTagStruct> blueHPSTags =
      List.of(blueHPSDriverLeft, blueHPSDriverRight);
  public static final List<AprilTagStruct> redHPSTags =
      List.of(redHPSDriverLeft, redHPSDriverRight);
}
