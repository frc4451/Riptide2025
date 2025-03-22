package frc.robot.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.List;

public class FieldConstants {
  /** AdvantageKit-safe loggable version of `AprilTag` that contains data we want without lookups */
  public static record AprilTagStruct(int fiducialId, Pose3d pose) {}

  /** Distance from the center of the April Tag on the Face to the center of the Pole */
  public static final double tagToReef = Units.inchesToMeters(6.6);

  public static final double distanceToTag = Units.inchesToMeters(10);

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
  public static final AprilTagStruct blueBarge =
      new AprilTagStruct(14, VisionConstants.fieldLayout.getTagPose(14).get());

  // See Allred's sheet for why they're AB/CD/EF/etc
  public static final ReefFace blueReefAB =
      new ReefFace(new AprilTagStruct(18, VisionConstants.fieldLayout.getTagPose(18).get()));
  public static final ReefFace blueReefCD =
      new ReefFace(new AprilTagStruct(17, VisionConstants.fieldLayout.getTagPose(17).get()));
  public static final ReefFace blueReefEF =
      new ReefFace(new AprilTagStruct(22, VisionConstants.fieldLayout.getTagPose(22).get()));
  public static final ReefFace blueReefGH =
      new ReefFace(new AprilTagStruct(21, VisionConstants.fieldLayout.getTagPose(21).get()));
  public static final ReefFace blueReefIJ =
      new ReefFace(new AprilTagStruct(20, VisionConstants.fieldLayout.getTagPose(20).get()));
  public static final ReefFace blueReefKL =
      new ReefFace(new AprilTagStruct(19, VisionConstants.fieldLayout.getTagPose(19).get()));

  public static final AprilTagStruct redHPSDriverLeft =
      new AprilTagStruct(1, VisionConstants.fieldLayout.getTagPose(1).get());
  public static final AprilTagStruct redHPSDriverRight =
      new AprilTagStruct(2, VisionConstants.fieldLayout.getTagPose(2).get());

  public static final Integer redProcessor = 3;
  public static final AprilTagStruct redBarge =
      new AprilTagStruct(4, VisionConstants.fieldLayout.getTagPose(4).get());

  public static final ReefFace redReefAB =
      new ReefFace(new AprilTagStruct(7, VisionConstants.fieldLayout.getTagPose(7).get()));
  public static final ReefFace redReefCD =
      new ReefFace(new AprilTagStruct(8, VisionConstants.fieldLayout.getTagPose(8).get()));
  public static final ReefFace redReefEF =
      new ReefFace(new AprilTagStruct(9, VisionConstants.fieldLayout.getTagPose(9).get()));
  public static final ReefFace redReefGH =
      new ReefFace(new AprilTagStruct(10, VisionConstants.fieldLayout.getTagPose(10).get()));
  public static final ReefFace redReefIJ =
      new ReefFace(new AprilTagStruct(11, VisionConstants.fieldLayout.getTagPose(11).get()));
  public static final ReefFace redReefKL =
      new ReefFace(new AprilTagStruct(6, VisionConstants.fieldLayout.getTagPose(6).get()));

  public static final List<ReefFace> blueReefFaces =
      List.of(blueReefAB, blueReefCD, blueReefEF, blueReefGH, blueReefIJ, blueReefKL);
  public static final List<ReefFace> redReefFaces =
      List.of(redReefAB, redReefCD, redReefEF, redReefGH, redReefIJ, redReefKL);

  public static final List<AprilTagStruct> blueHPSTags =
      List.of(blueHPSDriverLeft, blueHPSDriverRight);
  public static final List<AprilTagStruct> redHPSTags =
      List.of(redHPSDriverLeft, redHPSDriverRight);

  public static final List<AprilTagStruct> blueReefTags =
      blueReefFaces.stream().map(reefFace -> reefFace.tag).toList();
  public static final List<AprilTagStruct> redReefTags =
      redReefFaces.stream().map(reefFace -> reefFace.tag).toList();
}
