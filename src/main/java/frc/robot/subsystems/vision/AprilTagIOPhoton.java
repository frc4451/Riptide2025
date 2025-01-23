package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
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

    // List<PoseObservation> allLocalizedPoseObservations = new ArrayList<>();
    // List<PhotonTrackedTarget> allTargets = List.of();
    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTags = new ArrayList<>();

    for (PhotonPipelineResult result : unreadResults) {
      // Filtering of tags by ambiguity threshold & validity of id
      // Does not apply to global pose estimation
      // List<PhotonTrackedTarget> filteredTargets =
      // result.getTargets().stream()
      // .filter(
      // target ->
      // target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff
      // && target.getFiducialId() != -1)
      // .toList();

      // allTargets.addAll(targets);

      // Detected Corners
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (AprilTagAlgorithms.isValid(target)) {
          // for (TargetCorner corner : target.getDetectedCorners()) {
          //   validCorners.add(new Translation2d(corner.x, corner.y));
          // }
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(validCorners::add);

          validIds.add(target.getFiducialId());

          validAprilTags.add(VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get());
        } else {
          // for (TargetCorner corner : target.getDetectedCorners()) {
          //   rejectedCorners.add(new Translation2d(corner.x, corner.y));
          // }
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(rejectedCorners::add);

          rejectedIds.add(target.getFiducialId());
        }
      }

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

        PoseObservation observation =
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                multiTagResult.estimatedPose.ambiguity,
                // multiTagResult.fiducialIDsUsed.stream().mapToInt(id -> id).toArray()
                -100,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

        validPoseObservations.add(observation);
        validPoses.add(observation.robotPose());
      } else if (!result.getTargets().isEmpty()) {
        PhotonTrackedTarget target = result.getTargets().get(0);

        PoseObservation observation =
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                target.poseAmbiguity,
                // new int[] {target.fiducialId}
                target.fiducialId,
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        if (AprilTagAlgorithms.isValid(target)) {
          validPoseObservations.add(observation);
          validPoses.add(observation.robotPose());
        } else {
          rejectedPoseObservations.add(observation);
          rejectedPoses.add(observation.robotPose());
        }
      }
    }

    inputs.connected = camera.isConnected();

    inputs.validCorners = validCorners.toArray(Translation2d[]::new);
    inputs.rejectedCorners = rejectedCorners.toArray(Translation2d[]::new);

    inputs.validIds = validIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.rejectedIds = rejectedIds.stream().mapToInt(Integer::intValue).toArray();

    inputs.validPoseObservations = validPoseObservations.toArray(PoseObservation[]::new);
    inputs.rejectedPoseObservations = rejectedPoseObservations.toArray(PoseObservation[]::new);

    inputs.validPoses = validPoses.toArray(Pose3d[]::new);
    inputs.rejectedPoses = rejectedPoses.toArray(Pose3d[]::new);

    inputs.validAprilTagPoses = validAprilTags.toArray(Pose3d[]::new);
  }
}
