package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimationMethod;
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
  private final PhotonPoseEstimator globalEstimator;
  private final PhotonPoseEstimator localEstimator;

  public AprilTagIOPhoton(VisionSource source) {
    camera = new PhotonCamera(source.name());

    globalEstimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            source.robotToCamera());

    globalEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    localEstimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            source.robotToCamera());
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<PoseObservation> validLocalPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedLocalPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTagPoses = new ArrayList<>();
    List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

    for (PhotonPipelineResult result : unreadResults) {
      // Detected Corners
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (AprilTagAlgorithms.isValid(target)) {
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(validCorners::add);

          validIds.add(target.getFiducialId());

          validAprilTagPoses.add(
              VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get());
        } else {
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(rejectedCorners::add);

          rejectedIds.add(target.getFiducialId());

          if (target.getFiducialId() != -1) {
            VisionConstants.fieldLayout
                .getTagPose(target.getFiducialId())
                .ifPresent(rejectedAprilTagPoses::add);
          }
        }

        // Pose Estimation
        calcGlobalPoseObservations(result, validPoseObservations, rejectedPoseObservations);
        calcLocalPoseObservations(
            result, validLocalPoseObservations, rejectedLocalPoseObservations);
      }
    }

    inputs.connected = camera.isConnected();

    inputs.validCorners = validCorners.toArray(Translation2d[]::new);
    inputs.rejectedCorners = rejectedCorners.toArray(Translation2d[]::new);

    inputs.validIds = validIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.rejectedIds = rejectedIds.stream().mapToInt(Integer::intValue).toArray();

    inputs.validPoseObservations = validPoseObservations.toArray(PoseObservation[]::new);
    inputs.rejectedPoseObservations = rejectedPoseObservations.toArray(PoseObservation[]::new);

    inputs.validLocalPoseObservations = validLocalPoseObservations.toArray(PoseObservation[]::new);
    inputs.rejectedLocalPoseObservations =
        rejectedLocalPoseObservations.toArray(PoseObservation[]::new);

    inputs.validPoses = validPoses.toArray(Pose3d[]::new);
    inputs.rejectedPoses = rejectedPoses.toArray(Pose3d[]::new);

    inputs.validAprilTagPoses = validAprilTagPoses.toArray(Pose3d[]::new);
    inputs.rejectedAprilTagPoses = rejectedAprilTagPoses.toArray(Pose3d[]::new);
  }

  @Override
  public void addHeadingData(double timestampSeconds, Rotation2d heading) {
    globalEstimator.addHeadingData(timestampSeconds, heading);
  }

  private void calcGlobalPoseObservations(
      PhotonPipelineResult result,
      List<PoseObservation> validPoseObservations,
      List<PoseObservation> rejectedPoseObservations) {
    Optional<EstimatedRobotPose> maybeEstimatedPose = globalEstimator.update(result);
    if (maybeEstimatedPose.isEmpty()) {
      return;
    }

    EstimatedRobotPose estimatedPose = maybeEstimatedPose.get();

    if (result.getMultiTagResult().isPresent()) {
      MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();

      Pose3d pose = estimatedPose.estimatedPose;
      Matrix<N3, N1> stdDevs =
          AprilTagAlgorithms.getEstimationStdDevs(pose.toPose2d(), result.getTargets());

      PoseObservation observation =
          new PoseObservation(
              estimatedPose.estimatedPose,
              estimatedPose.timestampSeconds,
              multiTagResult.estimatedPose.ambiguity,
              // multiTagResult.fiducialIDsUsed.stream().mapToInt(id -> id).toArray()
              -100,
              stdDevs,
              PoseEstimationMethod.MULTI_TAG);

      validPoseObservations.add(observation);
    } else if (!result.getTargets().isEmpty()) {
      PhotonTrackedTarget target = result.getTargets().get(0);

      Pose3d pose = estimatedPose.estimatedPose;
      Matrix<N3, N1> stdDevs =
          AprilTagAlgorithms.getEstimationStdDevs(pose.toPose2d(), result.getTargets());
      PoseObservation observation =
          new PoseObservation(
              estimatedPose.estimatedPose,
              estimatedPose.timestampSeconds,
              target.poseAmbiguity,
              // new int[] {target.fiducialId}
              target.fiducialId,
              stdDevs,
              PoseEstimationMethod.SINGLE_TAG);

      if (AprilTagAlgorithms.isValid(target)) {
        validPoseObservations.add(observation);
      } else {
        rejectedPoseObservations.add(observation);
      }
    }
  }

  private void calcLocalPoseObservations(
      PhotonPipelineResult result,
      List<PoseObservation> validLocalPoseObservations,
      List<PoseObservation> rejectedPoseObservations) {

    Optional<EstimatedRobotPose> maybeEstimatedPose = localEstimator.update(result);
    if (maybeEstimatedPose.isEmpty()) {
      return;
    }

    EstimatedRobotPose estimatedPose = maybeEstimatedPose.get();

    PhotonTrackedTarget target = result.getBestTarget();

    PoseObservation observation =
        new PoseObservation(
            estimatedPose.estimatedPose,
            estimatedPose.timestampSeconds,
            target.poseAmbiguity,
            target.fiducialId,
            VisionConstants.trustedStdDevs,
            PoseEstimationMethod.TRIG);

    if (AprilTagAlgorithms.isValid(target)) {
      validLocalPoseObservations.add(observation);
    } else {
      rejectedPoseObservations.add(observation);
    }
  }
}
