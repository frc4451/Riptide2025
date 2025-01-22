package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.VisionSource;
import java.util.Arrays;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagPhotonSim implements AprilTagIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private final DuplicateTracker duplicateTracker = new DuplicateTracker();

  private final NetworkTableEntry heartbeatEntry;

  private PhotonCameraSim cameraSim;

  private final Thread periodicThread =
      new Thread(
          () -> {
            while (!Thread.currentThread().isInterrupted()) {
              periodic();

              try {
                Thread.sleep(10);
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            }
          });

  public AprilTagPhotonSim(VisionSource source) {
    camera = new PhotonCamera(source.name());

    estimator =
        new PhotonPoseEstimator(
            VisionConstants.FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            source.robotToCamera());

    estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    heartbeatEntry =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable(source.name())
            .getEntry("heartbeat");

    VisionConstants.VISION_SYSTEM_SIM.ifPresent(
        (visionSim) -> {
          SimCameraProperties simCameraProperties = new SimCameraProperties();

          // All of our AprilTag cameras use 16:9 FHD resolution
          simCameraProperties.setCalibration(source.sensorWithLens().x(), source.sensorWithLens().y(), Rotation2d.fromDegrees(source.sensorWithLens().fov()));
          // I read this from the docs, but this may need adjusting
          simCameraProperties.setCalibError(0.25, 0.08);
          simCameraProperties.setFPS(60.0);
          simCameraProperties.setAvgLatencyMs(20);
          simCameraProperties.setLatencyStdDevMs(5);

          cameraSim = new PhotonCameraSim(camera, simCameraProperties);

          visionSim.addCamera(cameraSim, source.robotToCamera());
          cameraSim.enableDrawWireframe(true);
          cameraSim.setMaxSightRange(10.0);
          cameraSim.setWireframeResolution(1);
        });

    periodicThread.setPriority(Thread.MAX_PRIORITY);
    periodicThread.start();
  }

  private PhotonPipelineResult frame = new PhotonPipelineResult();
  private boolean isDuplicateFrame = false;
  private EstimatedPose estimatedPose = new EstimatedPose();
  private int heartbeat = 0;

  private void periodic() {
    heartbeat = (int) heartbeatEntry.getInteger(-1);

    PhotonPipelineResult frame = camera.getLatestResult();

    isDuplicateFrame = duplicateTracker.isDuplicateFrame(frame);

    if (isDuplicateFrame) {
      return;
    }

    AprilTagFiltering.removeTooFarTargets(frame);
    this.frame = frame;
    estimatedPose = new EstimatedPose(AprilTagAlgorithms.estimateRobotPose(frame, estimator));

    updateFieldPoseEstimate();
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.frame = frame;
    inputs.isDuplicateFrame = isDuplicateFrame;
    // inputs.estimatedPose = estimatedPose;
    inputs.isConnected = camera.isConnected();
    inputs.heartbeat = heartbeat;

    inputs.visibleIds =
        inputs.frame.getTargets().stream()
            .filter(target -> VisionConstants.ALL_TAGS.contains(target.getFiducialId()))
            .mapToInt(PhotonTrackedTarget::getFiducialId)
            .toArray();

    inputs.visiblePoses =
        Arrays.stream(inputs.visibleIds)
            .boxed()
            .map(id -> VisionConstants.FIELD_LAYOUT.getTagPose(id).get())
            .toArray(Pose3d[]::new);

    // Detected Corners
    // List<Translation2d> corners = List.of();

    // inputs.frame.getTargets().stream()
    //     .map(target -> target.getDetectedCorners())
    //     .flatMap(List::stream)
    //     .map(corner -> new Translation2d(corner.x, corner.y))
    //     .forEach(translation -> corners.add(translation));

    // inputs.corners = corners.stream().toArray(Translation2d[]::new);

    inputs.corners =
        inputs.frame.getTargets().stream()
            .flatMap(target -> target.getDetectedCorners().stream())
            .map(corner -> new Translation2d(corner.x, corner.y))
            .toArray(Translation2d[]::new);
  }

  public void updateFieldPoseEstimate() {
    VisionConstants.VISION_SYSTEM_SIM.ifPresent(
        (visionSystemSim) -> {
          FieldObject2d visionEstimation =
              visionSystemSim.getDebugField().getObject("VisionEstimation");

          if (estimatedPose.isPresent) {
            visionEstimation.setPoses(estimatedPose.pose.toPose2d());
          } else {
            visionEstimation.setPoses();
          }
        });
  }
}
