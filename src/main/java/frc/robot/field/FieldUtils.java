package frc.robot.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldConstants.AprilTagStruct;
import java.util.List;

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

  public static ReefFace getClosestReef() {
    Translation2d robotTranslation = BobotState.getGlobalPose().getTranslation();

    return getReefFaces().stream()
        .reduce(
            (ReefFace reef1, ReefFace reef2) ->
                robotTranslation.getDistance(reef1.tag.pose().getTranslation().toTranslation2d())
                        < robotTranslation.getDistance(
                            reef2.tag.pose().getTranslation().toTranslation2d())
                    ? reef1
                    : reef2)
        .get();
  }

  public static AprilTagStruct getClosestHPSTag() {
    List<AprilTagStruct> hpsTags =
        FieldUtils.isBlueAlliance() ? FieldConstants.blueHPSTags : FieldConstants.redHPSTags;

    Translation2d robotTranslation = BobotState.getGlobalPose().getTranslation();

    AprilTagStruct closestTag =
        hpsTags.stream()
            .reduce(
                (AprilTagStruct tag1, AprilTagStruct tag2) ->
                    robotTranslation.getDistance(tag1.pose().getTranslation().toTranslation2d())
                            < robotTranslation.getDistance(
                                tag2.pose().getTranslation().toTranslation2d())
                        ? tag1
                        : tag2)
            .get();

    return closestTag;
  }

  public static AprilTagStruct getBargeTag() {
    return FieldUtils.isBlueAlliance() ? FieldConstants.blueBarge : FieldConstants.redBarge;
  }

  public static List<ReefFace> getReefFaces() {
    return FieldUtils.isBlueAlliance() ? FieldConstants.blueReefFaces : FieldConstants.redReefFaces;
  }

  public static List<Integer> getReefIDs() {
    return getReefFaces().stream().map(reef -> reef.tag.fiducialId()).toList();
  }
}
