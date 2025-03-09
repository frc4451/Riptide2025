package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagAlgorithms {
  /**
   * The standard deviations of the estimated pose for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets The targets used in the calc for the pose. Must be a non-zero amount.
   * @return The calculated standard deviations. Or empty if not suitable for estimation.
   * @apiNote Calc is short for calculator by the way.
   * @apiNote I'm just using slang guys.
   */
  public static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double totalDistance = 0;
    for (PhotonTrackedTarget target : targets) {
      var tagPose = VisionConstants.fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;

      numTags++;
      totalDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    double avgDistance = totalDistance / numTags;

    // Decrease std deviations further if more than one target is available
    Matrix<N3, N1> stdDevs =
        numTags == 1 ? VisionConstants.singleTagStdDevs : VisionConstants.multiTagStdDevs;

    // Increase std devs based on average distance
    stdDevs = stdDevs.times(1 + (avgDistance * avgDistance / 30.0));

    return stdDevs;
  }
}
