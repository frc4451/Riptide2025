package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public class ProcessorFace {
  public final AprilTagStruct tag;
  public final ProcessorLocation processor;

  public ProcessorFace(AprilTagStruct tag) {
    this.tag = tag;
    this.processor = new ProcessorLocation(tag);
  }

  public double getPerpendicularError(Pose2d robotPose) {
    return PoseUtils.getPerpendicularError(robotPose, tag.pose().toPose2d());
  }
}
