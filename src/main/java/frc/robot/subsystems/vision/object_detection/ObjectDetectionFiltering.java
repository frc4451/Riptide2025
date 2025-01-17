package frc.robot.subsystems.vision.object_detection;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionFiltering {
  private ObjectDetectionFiltering() {}

  /** Takes a frame and collect targets that are not April Tags */
  public static List<PhotonTrackedTarget> getNonFiducialTargets(PhotonPipelineResult frame) {
    return frame.targets.stream().filter(target -> target.getFiducialId() == -1).toList();
  }
}
