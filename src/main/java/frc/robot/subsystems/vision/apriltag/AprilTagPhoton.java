package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.VisionSource;
import java.util.Arrays;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagPhoton implements AprilTagIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private final DuplicateTracker duplicateTracker = new DuplicateTracker();

  private final IntegerSubscriber heartbeatEntry;

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

  public AprilTagPhoton(VisionSource source) {
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
            .getIntegerTopic("heartbeat")
            .subscribe(-1);

    periodicThread.setPriority(Thread.MAX_PRIORITY);
    periodicThread.start();
  }

  private PhotonPipelineResult frame = new PhotonPipelineResult();
  private boolean isDuplicateFrame = false;
  private EstimatedPose estimatedPose = new EstimatedPose();
  private boolean isConnected = false;
  private int heartbeat = 0;

  private void periodic() {
    isConnected = camera.isConnected();
    heartbeat = (int) heartbeatEntry.get();

    if (!isConnected) {
      frame = new PhotonPipelineResult();
      isDuplicateFrame = false;
      estimatedPose = new EstimatedPose();
      return;
    }

    PhotonPipelineResult latestFrame = camera.getLatestResult();

    isDuplicateFrame = duplicateTracker.isDuplicateFrame(latestFrame);

    if (isDuplicateFrame) {
      return;
    }

    AprilTagFiltering.removeTooFarTargets(latestFrame);
    frame = latestFrame;
    estimatedPose = new EstimatedPose(AprilTagAlgorithms.estimateRobotPose(latestFrame, estimator));
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.frame = frame;
    inputs.isDuplicateFrame = isDuplicateFrame;
    inputs.estimatedPose = estimatedPose;
    inputs.isConnected = isConnected;
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
  }
}
