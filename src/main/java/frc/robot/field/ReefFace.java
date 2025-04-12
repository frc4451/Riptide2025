package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public class ReefFace {
  public final AprilTagStruct tag;
  public final ReefPole center;
  public final ReefPole leftPole;
  public final ReefPole rightPole;
  public final boolean isL2Algae;

  public final Pose2d leftL1;
  public final Pose2d rightL1;

  public ReefFace(AprilTagStruct tag, boolean isL2Algae) {
    this.tag = tag;
    this.center = new ReefPole(tag, 0.0);
    this.leftPole = new ReefPole(tag, FieldConstants.eventConstants.tagToReefLeft);
    this.rightPole = new ReefPole(tag, FieldConstants.eventConstants.tagToReefRight);
    this.isL2Algae = isL2Algae;

    this.leftL1 = tag.pose().toPose2d().transformBy(FieldConstants.eventConstants.leftL1);
    this.rightL1 = tag.pose().toPose2d().transformBy(FieldConstants.eventConstants.rightL1);
  }

  public double getPerpendicularError(Pose2d robotPose) {
    return PoseUtils.getPerpendicularError(robotPose, tag.pose().toPose2d());
  }
}
