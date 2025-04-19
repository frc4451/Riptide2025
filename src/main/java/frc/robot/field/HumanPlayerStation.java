package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import java.util.stream.Stream;

public class HumanPlayerStation {
  public final AprilTagStruct tag;
  public final Pose2d left;
  public final Pose2d center;
  public final Pose2d right;

  public HumanPlayerStation(AprilTagStruct tag) {
    Pose2d tagPose = tag.pose().toPose2d();

    this.tag = tag;
    this.left =
        tagPose.transformBy(
            new Transform2d(
                FieldConstants.eventConstants.hpsOffset,
                FieldConstants.eventConstants.hpsSideOffset,
                Rotation2d.kZero));

    this.center =
        tagPose.transformBy(
            new Transform2d(FieldConstants.eventConstants.hpsOffset, 0, Rotation2d.kZero));

    this.right =
        tagPose.transformBy(
            new Transform2d(
                FieldConstants.eventConstants.hpsOffset,
                -FieldConstants.eventConstants.hpsSideOffset,
                Rotation2d.kZero));
  }

  public Pose2d getClosest(Pose2d pose) {
    Translation2d translation = pose.getTranslation();

    return Stream.of(left, center, right)
        .reduce(
            (Pose2d a, Pose2d b) ->
                translation.getDistance(a.getTranslation())
                        < translation.getDistance(b.getTranslation())
                    ? a
                    : b)
        .get();
  }
}
