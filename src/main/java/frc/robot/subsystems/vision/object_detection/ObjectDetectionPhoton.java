package frc.robot.subsystems.vision.object_detection;

import frc.robot.subsystems.vision.VisionConstants.VisionSource;
import frc.robot.subsystems.vision.apriltag.DuplicateTracker;
import frc.robot.util.TimeSinceConditionTracker;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class ObjectDetectionPhoton implements ObjectDetectionIO {
  private final PhotonCamera camera;
  private final DuplicateTracker duplicateTracker = new DuplicateTracker();
  private final TimeSinceConditionTracker timeSinceTargetsLostTracker;

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

  public ObjectDetectionPhoton(VisionSource source) {
    camera = new PhotonCamera(source.name());

    timeSinceTargetsLostTracker = new TimeSinceConditionTracker(() -> !frame.hasTargets(), 0.3);

    periodicThread.setPriority(Thread.MAX_PRIORITY);
    periodicThread.start();
  }

  private PhotonPipelineResult frame = new PhotonPipelineResult();
  private boolean isDuplicateFrame = false;
  private boolean hasExceededTargetsLostThreshold = false;

  private void periodic() {
    PhotonPipelineResult latestFrame = camera.getLatestResult();

    isDuplicateFrame = duplicateTracker.isDuplicateFrame(latestFrame);

    if (isDuplicateFrame) {
      return;
    }

    frame = camera.getLatestResult();
    timeSinceTargetsLostTracker.update(frame.getTimestampSeconds());
    hasExceededTargetsLostThreshold = timeSinceTargetsLostTracker.hasExceededThreshold();
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.frame = frame;
    inputs.isDuplicateFrame = isDuplicateFrame;
    inputs.hasExceededTargetlessThreshold = hasExceededTargetsLostThreshold;
    inputs.isConnected = camera.isConnected();
  }
}
