package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseUtils {
  public static Pose2d getParallelOffsetPose(Pose2d pose, double offsetMeters) {
    Translation2d offsetTranslation =
        pose.getTranslation()
            .plus(
                new Translation2d(
                    // Add 90 degrees to all trig functions
                    // so it is offset parallel to the face of the tag
                    -offsetMeters * pose.getRotation().getSin(),
                    offsetMeters * pose.getRotation().getCos()));

    return new Pose2d(offsetTranslation, pose.getRotation());
  }

  public static Pose2d getPerpendicularOffsetPose(Pose2d pose, double perpendicularOffset) {
    Translation2d offsetTranslation =
        pose.getTranslation()
            .plus(
                new Translation2d(
                    perpendicularOffset * pose.getRotation().getCos(),
                    perpendicularOffset * pose.getRotation().getSin()));

    return new Pose2d(offsetTranslation, pose.getRotation());
  }

  public static Pose2d getOffsetPose(
      Pose2d pose, double parellelOffsetMeters, double perpendicularOffsetMeters) {
    return getPerpendicularOffsetPose(
        getParallelOffsetPose(pose, parellelOffsetMeters), perpendicularOffsetMeters);
  }
}
