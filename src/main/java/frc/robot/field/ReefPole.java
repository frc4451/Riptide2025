package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public record ReefPole(AprilTagStruct tag, double tagToReef) {
  public Pose2d getPose() {
    return PoseUtils.getParallelOffsetPose(tag.pose().toPose2d(), tagToReef);
  }

  public double getParallellError(Pose2d robotPose) {
    return PoseUtils.getParallelError(robotPose, tag.pose().toPose2d());
  }
}
