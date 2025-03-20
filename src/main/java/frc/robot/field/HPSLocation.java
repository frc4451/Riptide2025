package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public record HPSLocation(AprilTagStruct tag) {
  public Pose2d getPose() {
    return PoseUtils.getParallelOffsetPose(tag.pose().toPose2d());
  }

  public double getParallellError(Pose2d robotPose) {
    return PoseUtils.getParallelError(robotPose, tag.pose().toPose2d());
  }

  public Pose2d getPerpendicularOffsetPose(double offsetMeters) {
    return PoseUtils.getPerpendicularOffsetPose(getPose(), offsetMeters);
  }
}
