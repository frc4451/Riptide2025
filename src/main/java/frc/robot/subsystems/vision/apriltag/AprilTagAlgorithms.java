package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;
import java.util.Optional;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagAlgorithms {
  /**
   * Create a {@link VisionMeasurement} with a pose & confidence value from an {@link
   * EstimatedRobotPose}
   */
  public static Optional<VisionMeasurement> findVisionMeasurement(
      EstimatedRobotPose estimatedPose) {
    // Empty if we only have one target, and it's not good enough to read
    if (estimatedPose.targetsUsed.size() == 1
        && (estimatedPose.targetsUsed.get(0).getPoseAmbiguity()
                > VisionConstants.POSE_AMBIGUITY_CUTOFF
            || estimatedPose.targetsUsed.get(0).getPoseAmbiguity() == -1)) {
      return Optional.empty();
    }

    // Remove targets that are not April Tags when calculating estimated pose
    estimatedPose.targetsUsed.removeIf(target -> target.getFiducialId() == -1);

    // Calculates the sums of every distance using Euclidiean Norm.
    // Think of it as the Pythagorean theorem except in three dimensions.
    double sumDistance =
        estimatedPose.targetsUsed.stream()
            .mapToDouble(
                target -> {
                  Transform3d targetPosition = target.getBestCameraToTarget();
                  return Math.sqrt(
                      Math.pow(targetPosition.getX(), 2)
                          + Math.pow(targetPosition.getY(), 2)
                          + Math.pow(targetPosition.getZ(), 2));
                })
            .sum();

    double averageDistance = sumDistance / estimatedPose.targetsUsed.size();

    // Calculate our 3x1 confidence matrix by clamping the number of targets
    // against the number of targets we see vs the number of targets we
    // can reliably calculate against.
    Matrix<N3, N1> confidence =
        VisionConstants.TAG_COUNT_DEVIATION_PARAMS
            .get(
                MathUtil.clamp(
                    estimatedPose.targetsUsed.size() - 1,
                    0,
                    VisionConstants.TAG_COUNT_DEVIATION_PARAMS.size() - 1))
            .computeDeviation(averageDistance);

    return Optional.of(new VisionMeasurement(estimatedPose, confidence));
  }

  public static Optional<EstimatedRobotPose> estimateRobotPose(
      PhotonPipelineResult frame, PhotonPoseEstimator estimator) {
    // Check if our frame has invalid targets
    if (AprilTagFiltering.shouldIgnoreFrame(frame, VisionConstants.ALL_TAGS)) {
      return Optional.empty();
    } else {
      return estimator.update(frame);
    }
  }

  public static Stream<TargetWithSource> filterTags(
      Stream<TargetWithSource> stream, int targetFiducialId) {
    return stream.filter(
        targetWithSource -> targetWithSource.target().getFiducialId() == targetFiducialId);
  }

  public static Optional<TargetWithSource> reduceToLeastAmbiguous(Stream<TargetWithSource> stream) {
    return stream.reduce(
        (targetWithSourceA, targetWithSourceB) ->
            targetWithSourceA.target().getPoseAmbiguity()
                    <= targetWithSourceB.target().getPoseAmbiguity()
                ? targetWithSourceA
                : targetWithSourceB);
  }
}
