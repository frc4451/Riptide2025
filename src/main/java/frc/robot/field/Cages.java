package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.field.FieldConstants.AprilTagStruct;

public class Cages {
  public final AprilTagStruct tag;

  public final Pose2d left;
  public final Pose2d center;
  public final Pose2d right;

  public final Pose2d shot;

  public Cages(AprilTagStruct tag) {
    this.tag = tag;

    this.center =
        tag.pose()
            .toPose2d()
            .transformBy(
                new Transform2d(FieldConstants.eventConstants.cageBackOffset, 0, Rotation2d.kZero));
    this.left =
        this.center.transformBy(
            new Transform2d(0, -FieldConstants.eventConstants.cageSideOffset, Rotation2d.kZero));
    this.right =
        this.center.transformBy(
            new Transform2d(0, FieldConstants.eventConstants.cageSideOffset, Rotation2d.kZero));

    this.shot =
        tag.pose()
            .toPose2d()
            .transformBy(
                new Transform2d(
                    FieldConstants.eventConstants.bargeShotOffset, 0, Rotation2d.kZero));
  }
}
