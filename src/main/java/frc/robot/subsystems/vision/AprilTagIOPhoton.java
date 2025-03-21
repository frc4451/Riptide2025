package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimationMethod;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  protected final PhotonCamera camera;

  /** This should compute our global pose we trust */
  private final PhotonPoseEstimator globalEstimator;

  /** This should compute our localized pose to targets we care about */
  private final PhotonPoseEstimator constrainedEstimator;

  private final Supplier<Rotation2d> headingSupplier;
  private final List<AprilTagStruct> trigConstrainedTargets;

  public AprilTagIOPhoton(
      VisionSource source,
      List<AprilTagStruct> trigConstrainedTargets,
      Supplier<Rotation2d> headingSupplier) {
    camera = new PhotonCamera(source.name());

    globalEstimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            source.robotToCamera());

    globalEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    constrainedEstimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            // See which one you like better
            // PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP,
            PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            source.robotToCamera());

    this.headingSupplier = headingSupplier;
    this.trigConstrainedTargets = trigConstrainedTargets;
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
      }

      // Constrained Pose Estimation

      // Heading is required for CONSTRAINED_SOLVEPNP and PNP_DISTANCE_TRIG_SOLVE
      constrainedEstimator.addHeadingData(result.getTimestampSeconds(), headingSupplier.get());
      Optional<EstimatedRobotPose> maybeConstrainedPose;

      /*
       * We have to provide camera intrinsics and distortion from Network Tables
       * to give the RIO the information needed to compute CONSTRAINED_SOLVEPNP.
       */
      if (constrainedEstimator.getPrimaryStrategy() == PoseStrategy.CONSTRAINED_SOLVEPNP) {
        maybeConstrainedPose =
            constrainedEstimator.update(
                result,
                camera.getCameraMatrix(),
                camera.getDistCoeffs(),
                VisionConstants.constrainedSolvePnpParams);
      } else if (constrainedEstimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
          && result.getBestTarget() != null
          && FieldUtils.getClosestReef().tag.fiducialId() == result.getBestTarget().fiducialId) {
        // && trigConstrainedTargets.stream()
        //     .anyMatch(id -> id.fiducialId() == result.getBestTarget().fiducialId)) {
        maybeConstrainedPose = constrainedEstimator.update(result);
      } else {
        maybeConstrainedPose = constrainedEstimator.update(result);
      }

      if (!maybeConstrainedPose.isPresent()) {
        continue;
      }

      EstimatedRobotPose estimatedConstrainedPose = maybeConstrainedPose.get();

      PoseObservation constrainedObservation =
          new PoseObservation(
              estimatedConstrainedPose.estimatedPose,
              estimatedConstrainedPose.timestampSeconds,
              VisionConstants
                  .noAmbiguity, // constrained observations use gyro heading as validation
              result.getBestTarget().getFiducialId(),
              VisionConstants.noStdDevs,
              PoseEstimationMethod.TRIG);

      validPoseObservations.add(constrainedObservation);
      validPoses.add(constrainedObservation.robotPose());

      // Global Pose Estimation
      Optional<EstimatedRobotPose> maybeEstimatedPose = globalEstimator.update(result);

      if (!maybeEstimatedPose.isPresent()) {
        continue;
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
                VisionConstants.noAmbiguity,
                stdDevs,
                PoseEstimationMethod.MULTI_TAG);

        validPoseObservations.add(observation);
        validPoses.add(observation.robotPose());
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

    inputs.validAprilTagPoses = validAprilTagPoses.toArray(Pose3d[]::new);
    inputs.rejectedAprilTagPoses = rejectedAprilTagPoses.toArray(Pose3d[]::new);
  }
}
