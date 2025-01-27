package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagAlgorithms {
  public static boolean isValid(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets The targets used in the calc for the pose.
   * @return The calculated standard deviations. Or empty if not suitable for estimation.
   * @apiNote Calc is short for calculator by the way.
   * @apiNote I'm just using slang guys.
   */
  public static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double avgDistance = 0;
    for (PhotonTrackedTarget target : targets) {
      var tagPose = VisionConstants.fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;

      numTags++;
      avgDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    Matrix<N3, N1> stdDevs = VisionConstants.singleTagStdDevs;
    if (numTags == 0) {
      return stdDevs;
    }

    avgDistance /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
      stdDevs = VisionConstants.multiTagStdDevs;
    }

    // Increase std devs based on average distance
    if (numTags == 1 && avgDistance > VisionConstants.singleTagPoseCutoffMeters) {
      // Too far for only one tag, throw away
      stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      stdDevs = stdDevs.times(1 + (avgDistance * avgDistance / 30.0));
    }

    return stdDevs;
  }
}
