package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static Pose2d getPerpendicularOffsetPose(Pose2d pose, double perpendicularOffsetMeters) {
    Translation2d offsetTranslation =
        pose.getTranslation()
            .plus(
                new Translation2d(
                    perpendicularOffsetMeters * pose.getRotation().getCos(),
                    perpendicularOffsetMeters * pose.getRotation().getSin()));

    return new Pose2d(offsetTranslation, pose.getRotation());
  }

  public static Pose2d getOffsetPose(
      Pose2d pose, double parellelOffsetMeters, double perpendicularOffsetMeters) {
    return getPerpendicularOffsetPose(
        getParallelOffsetPose(pose, parellelOffsetMeters), perpendicularOffsetMeters);
  }

  public static boolean isCloseToPose(Pose2d origin, Pose2d target, double distanceMeters) {
    return origin.minus(target).getTranslation().getNorm() < distanceMeters;
  }

  /**
   * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
   */
  public static double getParallelError(Pose2d origin, Pose2d target) {
    Translation2d originToTarget = origin.minus(target).getTranslation();
    Rotation2d angleBetween = originToTarget.getAngle();
    double parallelError = originToTarget.getNorm() * angleBetween.getSin();

    return parallelError;
  }

  /**
   * @see https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
   */
  public static double getPerpendicularError(Pose2d origin, Pose2d target) {
    Translation2d originToTarget = origin.minus(target).getTranslation();
    Rotation2d angleBetween = originToTarget.getAngle();
    double perpendicularError = originToTarget.getNorm() * angleBetween.getCos();

    return perpendicularError;
  }
}
