package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;

  public AprilTagIOPhoton(VisionSource source) {
    camera = new PhotonCamera(source.name());

    estimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            source.robotToCamera());

    estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    List<PoseObservation> poseObservations = List.of();
    List<PhotonTrackedTarget> targets = List.of();
    List<Translation2d> corners = List.of();

    for (PhotonPipelineResult result : unreadResults) {
      // Targets
      targets.addAll(result.getTargets());

      // Detected Corners
      result.getTargets().stream()
          .map(target -> target.getDetectedCorners())
          .flatMap(List::stream)
          .map(corner -> new Translation2d(corner.x, corner.y))
          .forEach(translation -> corners.add(translation));

      // Pose Estimation
      Optional<EstimatedRobotPose> maybeEstimatedPose = estimator.update(result);

      if (!maybeEstimatedPose.isPresent()) {
        continue;
      }

      EstimatedRobotPose estimatedPose = maybeEstimatedPose.get();

      if (result.getMultiTagResult().isPresent()) {
        MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();

        poseObservations.add(
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                multiTagResult.estimatedPose.ambiguity,
                multiTagResult.fiducialIDsUsed.stream().mapToInt(id -> id).toArray()));
      } else if (!result.getTargets().isEmpty()) {
        PhotonTrackedTarget target = result.getTargets().get(0);

        if (target.poseAmbiguity < VisionConstants.ambiguityCutoff) {
          poseObservations.add(
              new PoseObservation(
                  estimatedPose.estimatedPose,
                  estimatedPose.timestampSeconds,
                  target.poseAmbiguity,
                  new int[] {target.fiducialId}));
        }
      }
    }

    inputs.connected = camera.isConnected();

    inputs.poseObservations = poseObservations.stream().toArray(PoseObservation[]::new);
    // inputs.targets = targets.stream().toArray(PhotonTrackedTarget[]::new);
    inputs.corners = corners.stream().toArray(Translation2d[]::new);
  }
}
