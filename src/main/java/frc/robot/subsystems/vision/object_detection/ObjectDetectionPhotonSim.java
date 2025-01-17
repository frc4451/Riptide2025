package frc.robot.subsystems.vision.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.VisionSource;
import frc.robot.subsystems.vision.apriltag.DuplicateTracker;
import frc.robot.util.TimeSinceConditionTracker;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class ObjectDetectionPhotonSim implements ObjectDetectionIO {
  private final PhotonCamera camera;
  private PhotonCameraSim cameraSim;
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

  public ObjectDetectionPhotonSim(VisionSource source) {
    camera = new PhotonCamera(source.name());

    VisionConstants.VISION_SYSTEM_SIM.ifPresent(
        visionSim -> {
          // This creates a 10inx10inx2in rectangular prism,
          // but we eventually want to try making a more efficient
          // vertices object
          TargetModel notePlaceholder =
              new TargetModel(
                  Units.inchesToMeters(14), Units.inchesToMeters(14), Units.inchesToMeters(2));

          // TargetModel noteVertices = new TargetModel(
          // List.of(
          // new Translation3d(0.0, Units.inchesToMeters(-10), Units.inchesToMeters(-2)),
          // new Translation3d(0.0, Units.inchesToMeters(10), Units.inchesToMeters(2)),
          // new Translation3d(0.0, Units.inchesToMeters(10), Units.inchesToMeters(-2)),
          // new Translation3d(0.0, Units.inchesToMeters(20), Units.inchesToMeters(2)),
          // new Translation3d(0.0, Units.inchesToMeters(-20), Units.inchesToMeters(2))
          // ));

          visionSim.addVisionTargets(
              "note",
              new VisionTargetSim(new Pose3d(8.28, 2.45, 0.0, new Rotation3d()), notePlaceholder));

          SimCameraProperties simCameraProperties = new SimCameraProperties();

          // All of our AprilTag cameras use 16:9 FHD resolution
          simCameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
          // I read this from the docs, but this may need adjusting
          simCameraProperties.setCalibError(0.25, 0.08);
          simCameraProperties.setFPS(20.0);
          simCameraProperties.setAvgLatencyMs(35);
          simCameraProperties.setLatencyStdDevMs(5);

          cameraSim = new PhotonCameraSim(camera, simCameraProperties);

          visionSim.addCamera(cameraSim, source.robotToCamera());
          cameraSim.enableDrawWireframe(true);
          cameraSim.setMaxSightRange(10.0);
          cameraSim.setWireframeResolution(1);
        });

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
