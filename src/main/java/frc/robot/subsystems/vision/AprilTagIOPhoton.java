package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  protected final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private final Transform3d robotToCamera;

  public AprilTagIOPhoton(VisionSource source) {
    camera = new PhotonCamera(source.name());

    robotToCamera = source.robotToCamera();

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

    List<PoseObservation> allPoseObservations = new ArrayList<>();
    List<PoseObservation> allLocalizedPoseObservations = new ArrayList<>();
    // List<PhotonTrackedTarget> allTargets = List.of();
    List<Translation2d> allCorners = new ArrayList<>();

    for (PhotonPipelineResult result : unreadResults) {
      // Filtering of tags by ambiguity threshold & validity of id
      // Does not apply to global pose estimation
      // TODO: Rejections
      // List<PhotonTrackedTarget> filteredTargets =
      //     result.getTargets().stream()
      //         .filter(
      //             target ->
      //                 target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff
      //                     && target.getFiducialId() != -1)
      //         .toList();

      // allTargets.addAll(targets);

      // Detected Corners
      List<Translation2d> corners =
          result.getTargets().stream()
              .map(target -> target.getDetectedCorners())
              .flatMap(List::stream)
              .map(corner -> new Translation2d(corner.x, corner.y))
              .toList();

      allCorners.addAll(corners);

      // List<PoseObservation> localizedPoseObservations =
      // filteredTargets.stream()
      // .map(
      // target -> {
      // Transform3d robotToTarget =
      // target.getBestCameraToTarget().plus(robotToCamera);
      // Pose3d tagPose =
      // VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get();
      // Pose3d localizedRobotPose = tagPose.transformBy(robotToTarget.inverse());
      // // TODO: account for gyro
      //
      // return new PoseObservation(
      // localizedRobotPose,
      // result.getTimestampSeconds(),
      // target.getPoseAmbiguity(),
      // // new int[] {target.getFiducialId()}
      // target.getFiducialId(),
      // PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      // })
      // .toList();

      // allLocalizedPoseObservations.addAll(localizedPoseObservations);

      // Pose Estimation
      Optional<EstimatedRobotPose> maybeEstimatedPose = estimator.update(result);

      if (!maybeEstimatedPose.isPresent()) {
        continue;
      }

      EstimatedRobotPose estimatedPose = maybeEstimatedPose.get();

      if (result.getMultiTagResult().isPresent()) {
        MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();

        allPoseObservations.add(
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                multiTagResult.estimatedPose.ambiguity,
                // multiTagResult.fiducialIDsUsed.stream().mapToInt(id -> id).toArray()
                -100,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
      } else if (!result.getTargets().isEmpty()) {
        PhotonTrackedTarget target = result.getTargets().get(0);

        if (target.poseAmbiguity < VisionConstants.ambiguityCutoff
            && target.getFiducialId() != -1) {
          allPoseObservations.add(
              new PoseObservation(
                  estimatedPose.estimatedPose,
                  estimatedPose.timestampSeconds,
                  target.poseAmbiguity,
                  // new int[] {target.fiducialId}
                  target.fiducialId,
                  PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY));
        }
      }
    }

    inputs.connected = camera.isConnected();

    inputs.poseObservations = allPoseObservations.stream().toArray(PoseObservation[]::new);
    // inputs.targets = targets.stream().toArray(PhotonTrackedTarget[]::new);
    inputs.corners = allCorners.stream().toArray(Translation2d[]::new);
  }
}
